#pragma once

#include <string>

#include "model/order.h"
#include "../util/types.h"

class IObserver {
public:
    virtual ~IObserver() = default;
    virtual std::string getName() = 0;
    virtual void onEvent(int /*time*/, des::EventType /*type*/, const std::string& /*message*/, bool /*isDriving*/, bool /*isCharging*/, const std::string& /*color*/ = "", int /*missionId*/ = -1) {};
    virtual void onStateChanged(int /*time*/, const des::RobotStateType& /*type*/, const std::string& /*name*/, des::BatteryProps /*batStats*/) {};
    virtual void onMissionPublished(const des::OrderPtr& /*order*/, int /*time*/) {};
};
