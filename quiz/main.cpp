#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <map>
#include <set>
#include <cmath>      // for sqrt()
#include <iomanip>    // for setprecision()

using namespace std;

// ─── Point Structure ───────────────────────────────────────────
// Stores x, y coordinates of a point
// < operator: used for sorting (by x, then y for ties)
// == operator: used to detect and remove duplicate points
struct Point {
    double x, y;
    bool operator<(const Point& o) const {
        return x < o.x || (x == o.x && y < o.y);
    }
    bool operator==(const Point& o) const {
        return x == o.x && y == o.y;
    }
};

// ─── Cross Product (Orientation Test) ─────────────────────────
// Given three points O, A, B — determines their turn direction
// Return > 0 : Counter-clockwise turn (left)  → keep on hull
// Return = 0 : Collinear (all on same line)   → remove
// Return < 0 : Clockwise turn (right)         → remove from hull
double cross(const Point& O, const Point& A, const Point& B) {
    return (A.x - O.x) * (B.y - O.y)
         - (A.y - O.y) * (B.x - O.x);
}

// ─── Euclidean Distance ────────────────────────────────────────
// Calculates straight-line distance between two points P and Q
// Formula: sqrt((Qx - Px)^2 + (Qy - Py)^2)
double dist(const Point& P, const Point& Q) {
    double dx = Q.x - P.x;
    double dy = Q.y - P.y;
    return sqrt(dx * dx + dy * dy);
}

// ─── Andrew's Monotone Chain — Convex Hull ─────────────────────
// Finds the smallest convex polygon enclosing all given points
// Step 1: Sort points left to right (by x, then y)
// Step 2: Build lower hull (left → right)
// Step 3: Build upper hull (right → left)
// Step 4: Combine both halves → full convex hull
// Time complexity: O(n log n) due to sorting
vector<Point> convexHull(vector<Point> pts) {
    int n = pts.size();
    if (n < 2) return pts;

    // Sort points and remove duplicates
    sort(pts.begin(), pts.end());
    pts.erase(unique(pts.begin(), pts.end()), pts.end());
    n = pts.size();
    if (n < 2) return pts;

    vector<Point> hull;
    hull.reserve(2 * n);

    // Build lower hull (left to right)
    // If adding pts[i] causes a right turn or straight line,
    // pop the last point — it's not on the convex boundary
    for (int i = 0; i < n; i++) {
        while (hull.size() >= 2 &&
               cross(hull[hull.size()-2], hull[hull.size()-1], pts[i]) <= 0)
            hull.pop_back();
        hull.push_back(pts[i]);
    }

    // Build upper hull (right to left)
    // lower_size marks where lower hull ends
    // so we don't accidentally pop lower hull points
    int lower_size = hull.size() + 1;
    for (int i = n - 2; i >= 0; i--) {
        while ((int)hull.size() >= lower_size &&
               cross(hull[hull.size()-2], hull[hull.size()-1], pts[i]) <= 0)
            hull.pop_back();
        hull.push_back(pts[i]);
    }

    // Last point == first point, remove the duplicate
    hull.pop_back();
    return hull;
}

// ─── Perimeter of Convex Hull ──────────────────────────────────
// Sum of distances between consecutive hull vertices
// Last vertex connects back to first vertex (closed loop)
double perimeter(const vector<Point>& hull) {
    double total = 0.0;
    int n = hull.size();
    for (int i = 0; i < n; i++) {
        total += dist(hull[i], hull[(i + 1) % n]);
    }
    return total;
}

// ─── Main ──────────────────────────────────────────────────────
int main() {
    ios::sync_with_stdio(false); // Fast input
    cin.tie(nullptr);

    int N;
    cin >> N; // Number of rectangles

    vector<Point> points;
    points.reserve(4 * N); // Each rectangle has 4 corners

    for (int i = 0; i < N; i++) {
        double x1, y1, x2, y2;
        cin >> x1 >> y1 >> x2 >> y2;

        // Derive all 4 corners from the 2 opposite corners given
        // (x1,y1) ── (x2,y1)
        //    |            |
        // (x1,y2) ── (x2,y2)
        points.push_back({x1, y1});
        points.push_back({x2, y1});
        points.push_back({x1, y2});
        points.push_back({x2, y2});
    }

    // Compute convex hull of all rectangle corners
    vector<Point> hull = convexHull(points);

    // Output the perimeter (minimum patrol path length)
    // setprecision(9) ensures error <= 10^-6 as required
    cout << fixed << setprecision(9) << perimeter(hull) << "\n";

    return 0;
}