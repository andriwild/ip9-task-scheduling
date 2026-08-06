#pragma once

#include <ostream>
#include <vector>

namespace des {

// TODO: move to types and delete file
struct Point {
    double m_x, m_y, m_yaw;
    Point() = default;

    Point(const double pnt, const double pnt1, const double yaw) : m_x(pnt), m_y(pnt1), m_yaw(yaw) {}

    friend std::ostream& operator<<(std::ostream& os, const Point& s) {
        os << "(" << s.m_x << ", " << s.m_y << ", " << s.m_yaw << ")";
        return os;
    }
};

using Polygon = std::vector<Point>;

}  // namespace des
