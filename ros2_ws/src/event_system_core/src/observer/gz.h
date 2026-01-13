#pragma once

#include <string>
#include <map>

#include "../observer/observer.h"
#include "../util/types.h"
#include "../sim/gz_lib.h"

class GazeboView: public IObserver {
    std::map<std::string, des::Point> locationMap;

public:
    explicit GazeboView(std::map<std::string, des::Point> locationMap):
        locationMap(locationMap)
    {}

    void onRobotMoved(int time, const std::string& location, double distance) override {
        auto p = locationMap[location]; // TODO: robustness?
        sim::moveRobot(p.m_x, p.m_y);
    };
};
