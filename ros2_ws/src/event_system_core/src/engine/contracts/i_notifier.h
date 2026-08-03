#pragma once

#include "model/order.h"

class IEvent;

class INotifier {
public:
    virtual ~INotifier() = default;
    virtual void notifyEvent(const IEvent& event) const = 0;
    virtual void notifyBatteryChanged() const = 0;
    virtual void notifyChargeStarted() const = 0;
    virtual void publishMission(const des::OrderPtr& order, int time) = 0;
    virtual void publishMissionRegistered(const des::OrderPtr& order) = 0;
};
