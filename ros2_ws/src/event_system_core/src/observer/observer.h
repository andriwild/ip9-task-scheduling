#pragma once

#include <string>

#include "../util/types.h"

class IObserver {
public:
    virtual ~IObserver() = default;
    virtual std::string getName() = 0;
    virtual void onEvent(int /*time*/, des::EventType /*type*/, const std::string& /*message*/, bool /*isDriving*/, bool /*isCharging*/) {};
    virtual void onRobotMoved(int /*time*/, const std::string& /*location*/, double /*distance*/) {};
    virtual void onStateChanged(int /*time*/, const des::RobotStateType& /*type*/, des::BatteryProps /*batStats*/) {};
    virtual void onMissionComplete(int /*time*/, const des::MissionState& /*state*/, int /*timeDiff*/) {};
};
