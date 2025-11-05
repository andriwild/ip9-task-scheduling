#pragma once

#include <cmath>
#include <vector>
#include <unordered_map>
#include "../datastructure/graph.h"

namespace util {
    using PersonData = std::unordered_map<int, std::vector<int>>;

    inline double calculateDistance(const double x1, const double y1, const double x2, const double y2) {
        const double dx = x2 - x1;
        const double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

    inline int calcFromTo(const Node &from, const Node &to, const double speed) {
        const double dist = calculateDistance(from.x, from.y, to.x, to.y);
        return static_cast<int>(dist / speed);
    }

    inline std::vector<int> calcDriveTime(std::vector<int> pathIds, Graph &graph, double speed) {
        std::vector<int> driveTimes;
        for (int i = 0; i < pathIds.size() - 1; i ++) {
            int from = pathIds[i];
            int to = pathIds[i + 1];
            int time = calcFromTo(graph.getNode(from), graph.getNode(to), speed);
            driveTimes.push_back(time);
        }
        return driveTimes;
    }
}