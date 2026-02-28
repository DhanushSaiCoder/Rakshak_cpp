#pragma once
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace MathUtils {
    struct Point {
        double x;
        double y;
    };

    /**
     * @brief Performs linear interpolation between a set of points.
     * @param points Vector of points (x, y) sorted by x.
     * @param x The value to interpolate.
     * @return The interpolated y value.
     */
    inline double interpolateLinear(const std::vector<Point>& points, double x) {
        if (points.empty()) return 0.0;
        if (points.size() == 1) return points[0].y;

        // Ensure sorted by x
        // (Assuming they are sorted for performance, but we can clamp)
        if (x <= points.front().x) return points.front().y;
        if (x >= points.back().x) return points.back().y;

        auto it = std::lower_bound(points.begin(), points.end(), x, [](const Point& p, double val) {
            return p.x < val;
        });

        if (it == points.begin()) return points.front().y;
        
        const Point& p2 = *it;
        const Point& p1 = *std::prev(it);

        double t = (x - p1.x) / (p2.x - p1.x);
        return p1.y + t * (p2.y - p1.y);
    }
}
