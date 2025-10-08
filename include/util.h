#pragma once
#include <cmath>
#include "event.h"

namespace util {

    inline double calculateDistance(const Point& p1, const Point& p2) {
        const double dx = p2.x - p1.x;
        const double dy = p2.y - p1.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    inline int calcDriveTime(const Point &from, const Point &to, const double speed) {
        const double dist = calculateDistance(from, to);
        return static_cast<int>(dist / speed);
    }
}
