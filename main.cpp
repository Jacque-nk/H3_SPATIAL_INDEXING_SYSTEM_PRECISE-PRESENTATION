// ============================================================
//  H3-LIKE SPATIAL INDEXING SYSTEM
//  Integrated OOP Implementation (No external H3 library needed)
//
//  Components:
//    1. Location       — real-world GPS point
//    2. HexCell        — hexagonal grid cell (axial coords)
//    3. NeighborFinder — k-ring traversal logic
//    4. DistanceCalc   — Haversine + grid distance
//    5. Converter      — bridges Location → HexCell
// ============================================================

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace std;


// ============================================================
//  1. LOCATION CLASS
//     Represents a real-world GPS coordinate on Earth.
//     Encapsulates lat/lon/alt + validity + Haversine distance.
// ============================================================
class Location {
private:
    double latitude;
    double longitude;
    double altitude;
    string label;

    double toRadians(double degrees) const {
        return degrees * (3.14159265358979 / 180.0);
    }

public:
    // Constructor (altitude and label have defaults)
    Location(double lat, double lon, double alt = 0.0, string name = "Unknown")
        : latitude(lat), longitude(lon), altitude(alt), label(name) {}

    // ── Getters ─────────────────────────────────────────────
    double getLatitude()  const { return latitude;  }
    double getLongitude() const { return longitude; }
    double getAltitude()  const { return altitude;  }
    string getLabel()     const { return label;     }

    // ── Setter (only label is mutable after construction) ───
    void setLabel(string name) { label = name; }

    // ── isValid(): guard against impossible coordinates ─────
    bool isValid() const {
        return (latitude  >= -90.0  && latitude  <= 90.0) &&
               (longitude >= -180.0 && longitude <= 180.0);
    }

    // ── distanceTo(): Haversine formula (returns km) ─────────
    double distanceTo(const Location& other) const {
        const double R = 6371.0;
        double lat1 = toRadians(latitude),  lat2 = toRadians(other.latitude);
        double dLat = toRadians(other.latitude  - latitude);
        double dLon = toRadians(other.longitude - longitude);

        double a = sin(dLat/2)*sin(dLat/2) +
                   cos(lat1)*cos(lat2)*sin(dLon/2)*sin(dLon/2);
        return R * 2 * atan2(sqrt(a), sqrt(1 - a));
    }

    // ── display() ───────────────────────────────────────────
    void display() const {
        cout << "┌──────────────────────────────┐" << endl;
        cout << "│  Location : " << setw(18) << left << label << "│" << endl;
        cout << "├──────────────────────────────┤" << endl;
        cout << "│  Lat  : " << fixed << setprecision(4) << latitude  << " deg" << setw(9) << " │" << endl;
        cout << "│  Lon  : " << fixed << setprecision(4) << longitude << " deg" << setw(9) << " │" << endl;
        cout << "│  Alt  : " << fixed << setprecision(1) << altitude  << " m"   << setw(13) << " │" << endl;
        cout << "│  Valid: " << (isValid() ? "Yes" : "No ") << setw(22) << " │" << endl;
        cout << "└──────────────────────────────┘" << endl;
    }
};


// ============================================================
//  2. HEX CELL CLASS
//     Represents one hexagonal tile using axial coordinates.
//     Stores the originating GPS so we can mix both systems.
// ============================================================
class HexCell {
private:
    string   cellId;       // human label, e.g. "hex001"
    int      q;            // axial column
    int      r;            // axial row
    int      resolution;   // zoom level (like H3 res 0-15)
    double   originLat;    // source GPS latitude
    double   originLon;    // source GPS longitude
    vector<string> neighborIds;   // IDs of adjacent cells

public:
    // Constructor
    HexCell(string id, int res, int q, int r, double lat = 0.0, double lon = 0.0)
        : cellId(id), resolution(res), q(q), r(r), originLat(lat), originLon(lon) {}

    // ── Getters ─────────────────────────────────────────────
    string getId()         const { return cellId;     }
    int    getQ()          const { return q;          }
    int    getR()          const { return r;          }
    int    getResolution() const { return resolution; }
    double getOriginLat()  const { return originLat;  }
    double getOriginLon()  const { return originLon;  }
    vector<string> getNeighborIds() const { return neighborIds; }

    // ── Neighbor management ──────────────────────────────────
    void addNeighbor(const string& id) {
        neighborIds.push_back(id);
    }

    // ── Grid distance (hex steps via axial formula) ──────────
    int gridDistanceTo(const HexCell& other) const {
        if (resolution != other.resolution) {
            cout << "[HexCell] Warning: resolution mismatch ("
                 << resolution << " vs " << other.resolution << ")" << endl;
            return -1;
        }
        // Axial → cube coordinates, then Chebyshev distance
        int s1 = -q - r,  s2 = -other.q - other.r;
        return (abs(q - other.q) + abs(r - other.r) + abs(s1 - s2)) / 2;
    }

    // ── display() ───────────────────────────────────────────
    void display() const {
        cout << "HexCell [" << cellId << "]  res=" << resolution
             << "  axial=(" << q << "," << r << ")" << endl;
        cout << "  Origin GPS : (" << fixed << setprecision(4)
             << originLat << ", " << originLon << ")" << endl;
        cout << "  Neighbors  : ";
        if (neighborIds.empty()) {
            cout << "(none)";
        } else {
            for (const auto& n : neighborIds) cout << n << " ";
        }
        cout << endl;
    }
};


// ============================================================
//  3. NEIGHBOR FINDER CLASS
//     Static utility — finds immediate neighbors or k-rings.
//     Uses the 6-direction axial offset table.
// ============================================================
class NeighborFinder {
private:
    // The 6 axial directions in a flat-top hex grid
    static const vector<pair<int,int>> DIRECTIONS;

public:
    // ── Immediate ring (k = 1) ──────────────────────────────
    static vector<HexCell> getNeighbors(const HexCell& cell) {
        vector<HexCell> result;
        int n = 0;
        for (const auto& dir : DIRECTIONS) {
            int nq = cell.getQ() + dir.first;
            int nr = cell.getR() + dir.second;
            string nid = cell.getId() + "_n" + to_string(n++);
            result.emplace_back(nid, cell.getResolution(), nq, nr);
        }
        return result;
    }

    // ── k-ring: all cells within k steps ────────────────────
    static vector<HexCell> kRing(const HexCell& center, int k) {
        vector<HexCell> results;
        int idx = 0;
        for (int dq = -k; dq <= k; dq++) {
            int rMin = max(-k, -dq - k);
            int rMax = min( k, -dq + k);
            for (int dr = rMin; dr <= rMax; dr++) {
                int nq = center.getQ() + dq;
                int nr = center.getR() + dr;
                string nid = center.getId() + "_k" + to_string(idx++);
                results.emplace_back(nid, center.getResolution(), nq, nr);
            }
        }
        return results;
    }
};

// Static member definition
const vector<pair<int,int>> NeighborFinder::DIRECTIONS = {
    {1,0},{1,-1},{0,-1},{-1,0},{-1,1},{0,1}
};


// ============================================================
//  4. DISTANCE CALCULATOR CLASS
//     Centralises all distance logic in one place.
//     Supports both GPS (km) and grid (hex steps) distances.
// ============================================================
class DistanceCalculator {
public:
    // ── Haversine between two Locations ─────────────────────
    static double haversine(const Location& a, const Location& b) {
        return a.distanceTo(b);   // delegates to Location method
    }

    // ── Grid steps between two HexCells ─────────────────────
    static int gridSteps(const HexCell& a, const HexCell& b) {
        return a.gridDistanceTo(b);
    }

    // ── Summary report ──────────────────────────────────────
    static void report(const Location& locA, const Location& locB,
                       const HexCell& hexA, const HexCell& hexB) {
        cout << "\n╔══════════════════════════════════════╗" << endl;
        cout << "║       DISTANCE CALCULATOR REPORT     ║" << endl;
        cout << "╠══════════════════════════════════════╣" << endl;
        cout << "║  From : " << setw(29) << left << locA.getLabel() << "║" << endl;
        cout << "║  To   : " << setw(29) << left << locB.getLabel() << "║" << endl;
        cout << "╠══════════════════════════════════════╣" << endl;
        double km = haversine(locA, locB);
        int    gs = gridSteps(hexA, hexB);
        cout << "║  GPS  Distance : " << fixed << setprecision(2)
             << setw(8) << km << " km" << setw(9) << "║" << endl;
        cout << "║  Grid Distance : " << setw(8) << gs
             << " hex steps" << setw(5) << "║" << endl;
        cout << "╚══════════════════════════════════════╝" << endl;
    }
};


// ============================================================
//  5. CONVERTER CLASS
//     Bridges the Location (GPS) world → HexCell (grid) world.
//     Uses a simple flat-projection formula suitable for demo.
//     (Real H3 uses a full icosahedron projection.)
// ============================================================
class Converter {
private:
    // Scale factor: how many hex cells span 1 degree at each resolution
    static double cellsPerDegree(int resolution) {
        return pow(2.0, resolution);   // doubles each level, like H3
    }

public:
    // ── Convert Location → HexCell ──────────────────────────
    static HexCell fromLocation(const Location& loc, int resolution,
                                const string& id = "") {
        if (!loc.isValid()) {
            cout << "[Converter] ERROR: Location '" << loc.getLabel()
                 << "' has invalid coordinates. Returning origin cell." << endl;
            return HexCell(id.empty() ? "ERR" : id, resolution, 0, 0);
        }

        double scale = cellsPerDegree(resolution);
        int q = static_cast<int>(floor(loc.getLongitude() * scale));
        int r = static_cast<int>(floor(loc.getLatitude()  * scale));

        string cellId = id.empty() ? ("hex_" + loc.getLabel()) : id;

        cout << "[Converter] '" << loc.getLabel()
             << "' → HexCell(" << q << ", " << r
             << ") at res=" << resolution << endl;

        return HexCell(cellId, resolution, q, r,
                       loc.getLatitude(), loc.getLongitude());
    }
};


// ============================================================
//  MAIN — Ties every class together for demonstration
// ============================================================
int main() {

    cout << "\n========================================" << endl;
    cout << "  H3-LIKE SPATIAL INDEXING SYSTEM DEMO  " << endl;
    cout << "========================================\n" << endl;


    // ── STEP 1: Create Location objects ─────────────────────
    cout << "── [1] LOCATION OBJECTS ──────────────────\n" << endl;
    Location nairobi(-1.2921, 36.8219, 1661.0, "Nairobi CBD");
    Location jkuat  (-1.0917, 37.0144, 1548.0, "JKUAT Campus");
    Location invalid( 200.0,  400.0,   0.0,    "Impossible Place");

    nairobi.display();
    jkuat.display();
    cout << "Invalid location valid? "
         << (invalid.isValid() ? "Yes" : "No") << "\n" << endl;


    // ── STEP 2: Convert to HexCells via Converter ───────────
    cout << "── [2] CONVERTER: Location → HexCell ────\n" << endl;
    int RES = 5;
    HexCell hexNairobi = Converter::fromLocation(nairobi, RES, "hex001");
    HexCell hexJkuat   = Converter::fromLocation(jkuat,   RES, "hex002");
    HexCell hexBad     = Converter::fromLocation(invalid, RES, "hexERR");
    cout << endl;


    // ── STEP 3: Display HexCells ─────────────────────────────
    cout << "── [3] HEXCELL DISPLAY ──────────────────\n" << endl;
    hexNairobi.addNeighbor("hex002");
    hexNairobi.display();
    hexJkuat.display();
    cout << endl;


    // ── STEP 4: NeighborFinder ──────────────────────────────
    cout << "── [4] NEIGHBOR FINDER ──────────────────\n" << endl;

    cout << "Immediate neighbors of Nairobi CBD cell:" << endl;
    auto neighbors = NeighborFinder::getNeighbors(hexNairobi);
    for (auto& nb : neighbors) {
        cout << "  Cell (" << nb.getQ() << ", " << nb.getR() << ")" << endl;
    }

    cout << "\nK-ring (k=2) around Nairobi CBD — total cells: ";
    auto ring = NeighborFinder::kRing(hexNairobi, 2);
    cout << ring.size() << endl;
    cout << endl;


    // ── STEP 5: DistanceCalculator ──────────────────────────
    cout << "── [5] DISTANCE CALCULATOR ──────────────\n" << endl;
    DistanceCalculator::report(nairobi, jkuat, hexNairobi, hexJkuat);

    cout << "\n\nDemo complete." << endl;
    return 0;
}
