#pragma once
#include <cmath>
#include "../model/event.h"

namespace util {

    inline double calculateDistance(const Node& p1, const Node& p2) {
        const double dx = p2.x - p1.x;
        const double dy = p2.y - p1.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    inline int calcDriveTime(const Node &from, const Node &to, const double speed) {
        const double dist = calculateDistance(from, to);
        return static_cast<int>(dist / speed);
    }
}
