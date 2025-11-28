#pragma once

#include <string>

#include "../model/robot_state.h"

class IObserver {
public:
    virtual ~IObserver() = default;
    virtual void onLog(int time, const std::string& message) = 0;
    virtual void onRobotMoved(int time, const std::string& location) = 0;
    virtual void onStateChanged(int time, const RobotStateType type) {};
};
