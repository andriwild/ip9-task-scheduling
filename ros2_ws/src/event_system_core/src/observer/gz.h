#pragma once

#include <map>
#include <string>

#include "../observer/observer.h"
#include "../sim/gz_lib.h"
#include "../util/types.h"

class GazeboView final : public IObserver {
    std::map<std::string, des::Point> rooms;

public:
    std::string getName() override {
        return "Gazebo";
    }

    explicit GazeboView(const std::map<std::string, des::Point> &rooms) : rooms(rooms) {}

    void onRobotMoved(int /*time*/, const std::string& location, double /*distance*/) override {
        const auto p = rooms[location];  // TODO: robustness?
        sim::moveRobot(p.m_x, p.m_y);
    };

    void onRobotMovedTo(int /*time*/, const des::Point& position, double /*distance*/ = 0.0) override {
        sim::moveRobot(position.m_x, position.m_y);
    };
};
