#pragma once

#include <string>

#include "model/order.h"
#include "../util/types.h"

namespace des {

class IObserver {
public:
    virtual ~IObserver() = default;
    virtual std::string getName() = 0;
    virtual void onEvent(int /*time*/, EventType /*type*/, const std::string& /*message*/, bool /*isDriving*/, bool /*isCharging*/, const std::string& /*color*/ = "", int /*missionId*/ = -1) {};
    virtual void onStateChanged(int /*time*/, const RobotStateType& /*type*/, const std::string& /*name*/, BatteryProps /*batStats*/) {};
    virtual void onMissionPublished(const OrderPtr& /*order*/, int /*time*/) {};
};

}  // namespace des
