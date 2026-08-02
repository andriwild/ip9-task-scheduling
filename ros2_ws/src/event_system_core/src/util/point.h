#pragma once

#include <ostream>

namespace des {

struct Point {
    double m_x, m_y, m_yaw;
    Point() = default;

    Point(const double pnt, const double pnt1, const double yaw) : m_x(pnt), m_y(pnt1), m_yaw(yaw) {}

    friend std::ostream& operator<<(std::ostream& os, const Point& s) {
        os << "(" << s.m_x << ", " << s.m_y << ", " << s.m_yaw << ")";
        return os;
    }
};

struct Segment {
    int id;
    Point m_points[2];

    Segment() = default;

    Segment(const int segment_id, const Point& p1, const Point& p2) : id(segment_id), m_points{p1, p2} {}

    friend std::ostream& operator<<(std::ostream& os, const Segment& s) {
        os << "Segment id=" << s.id << ", p1=" << s.m_points[0] << ", p2=" << s.m_points[1];
        return os;
    }
};

}  // namespace des
