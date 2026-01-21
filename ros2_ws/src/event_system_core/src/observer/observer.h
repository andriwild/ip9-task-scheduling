#pragma once

#include <string>

#include "../model/robot_state.h"
#include "../util/types.h"

class IObserver {
public:
    virtual ~IObserver() = default;
    virtual std::string getName() = 0;
    virtual void onLog(int /*time*/, const std::string& /*message*/) {};
    virtual void onRobotMoved(int /*time*/, const std::string& /*location*/, double /*distance*/) {};
    virtual void onStateChanged(int /*time*/, const RobotStateType& /*type*/) {};
    virtual void onMissionComplete(int /*time*/, des::MissionState&, int /*timeDiff*/) {};
};
