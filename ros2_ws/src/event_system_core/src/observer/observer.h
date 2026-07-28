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
    virtual void onRobotMovedTo(int /*time*/, const des::Point& /*position*/, double /*distance*/ = 0.0) {};
    virtual void onStateChanged(int /*time*/, const des::RobotStateType& /*type*/, const std::string& /*name*/, des::BatteryProps /*batStats*/) {};
    virtual void onMissionComplete(int /*time*/, const des::OrderPtr& /*order*/, int /*timeDiff*/) {};
    virtual void onMissionPublished(const des::OrderPtr& /*order*/, int /*time*/) {};
    virtual void onMissionRegistered(const des::OrderPtr& /*order*/) {};
};
