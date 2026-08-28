#pragma once

#include "model/order.h"

namespace des {

class IEvent;

class INotifier {
public:
    virtual ~INotifier() = default;
    virtual void notifyEvent(const IEvent& event) const = 0;
    virtual void notifyRobotStateChanged() const = 0;
    virtual void notifyChargeStarted(ChargeTrigger trigger) const = 0;
    virtual void publishMission(const OrderPtr& order, int time) = 0;
};

}  // namespace des
