#pragma once

#include "path_node.h"
#include "../../util/types.h"

class TravelTimeEstimator {
private:
    std::shared_ptr<PathPlannerNode> planner;
    std::map<std::string, des::Point> locationMap;

public:
    TravelTimeEstimator(
        std::shared_ptr<PathPlannerNode> planner,
        std::map<std::string, des::Point> locationMap
    ):
        planner(planner), 
        locationMap(locationMap)
    {}

    std::optional<double> estimateDuration(const std::string& from, const std::string& to, const double speed) {
        auto fromIt = locationMap.find(from);
        auto toIt   = locationMap.find(to);

        assert(fromIt != locationMap.end() && toIt != locationMap.end());

        SimplePose start { fromIt->second.m_x, fromIt->second.m_y, 0.0 };
        SimplePose goal  { toIt->second.m_x, toIt->second.m_y, 0.0 };
        
        auto result = planner->computeDistance(start, goal);

        if (!result.success) return std::nullopt;

        return result.distance / speed;
    }
};
