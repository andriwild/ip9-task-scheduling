#pragma once

#include <map>
#include <string>

#include "../observer/observer.h"
#include "../sim/gz_lib.h"
#include "../util/types.h"

class GazeboView final : public IObserver {
    std::map<std::string, des::Point> locationMap;

public:
    std::string getName() override {
        return "Gazebo";
    }

    explicit GazeboView(const std::map<std::string, des::Point> &locationMap) : locationMap(locationMap) {}

    void onRobotMoved(int /*time*/, const std::string& location, double /*distance*/) override {
        const auto p = locationMap[location];  // TODO: robustness?
        sim::moveRobot(p.m_x, p.m_y);
    };
};
