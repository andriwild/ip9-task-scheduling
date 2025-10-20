#pragma once

#include <cmath>
#include <vector>
#include <unordered_map>
#include "../datastructure/graph.h"

namespace util {
    using PersonData = std::unordered_map<int, std::vector<int>>;

    inline double calculateDistance(const Node& p1, const Node& p2) {
        const double dx = p2.x - p1.x;
        const double dy = p2.y - p1.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    inline int calcFromTo(const Node &from, const Node &to, const double speed) {
        const double dist = calculateDistance(from, to);
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