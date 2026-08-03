#pragma once

#include <optional>
#include <vector>

#include "plugins/i_order.h"

class IScheduledMissions {
public:
    virtual ~IScheduledMissions() = default;
    virtual void addScheduledMission(const des::OrderPtr orderPtr) = 0;
    virtual bool hasScheduledMission() const = 0;
    virtual des::OrderPtr nextScheduledMission() = 0;
    virtual des::OrderPtr popScheduledMission() = 0;

    // Time of the next MissionDispatchEvent in the event queue, i.e. when the
    // next scheduled mission will become pending. nullopt = nothing scheduled.
    virtual std::optional<int> getNextScheduledDispatchTime() const = 0;

    // OrderPtr of the next scheduled mission still queued, or nullptr.
    virtual des::OrderPtr peekNextScheduledOrder() const = 0;

    virtual std::vector<des::OrderPtr> peekScheduledOrdersUntil(int untilTime) const = 0;
};
