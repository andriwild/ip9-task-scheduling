#pragma once

#include <string>

#include "../plugins/i_order.h"
#include "../util/types.h"

class IObserver {
public:
    virtual ~IObserver() = default;
    virtual std::string getName() = 0;
    virtual void onEvent(int /*time*/, des::EventType /*type*/, const std::string& /*message*/, bool /*isDriving*/, bool /*isCharging*/, const std::string& /*color*/ = "", int /*missionId*/ = -1) {};
    virtual void onRobotMoved(int /*time*/, const std::string& /*location*/, double /*distance*/) {};
    virtual void onStateChanged(int /*time*/, const des::RobotStateType& /*type*/, const std::string& /*name*/, des::BatteryProps /*batStats*/) {};
    virtual void onMissionComplete(int /*time*/, const des::MissionState& /*state*/, int /*timeDiff*/, des::ExecutionMode /*execution*/) {};
    virtual void onMissionPublished(const des::OrderPtr& /*order*/, int /*time*/) {};
    virtual void onMissionRegistered(const des::OrderPtr& /*order*/) {};
};
