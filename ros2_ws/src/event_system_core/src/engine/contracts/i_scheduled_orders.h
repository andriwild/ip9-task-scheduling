#pragma once

#include <optional>
#include <vector>

#include "model/order.h"

namespace des {

class IScheduledOrders {
public:
    virtual ~IScheduledOrders() = default;
    virtual void addScheduledOrder(const OrderPtr orderPtr) = 0;
    virtual bool hasScheduledOrder() const = 0;
    virtual OrderPtr nextScheduledOrder() = 0;
    virtual OrderPtr popScheduledOrder() = 0;

    // Time of the next MissionDispatchEvent in the event queue, i.e. when the
    // next scheduled mission will become pending. nullopt = nothing scheduled.
    virtual std::optional<int> getNextScheduledDispatchTime() const = 0;

    // OrderPtr of the next scheduled mission still queued, or nullptr.
    virtual OrderPtr peekNextScheduledOrder() const = 0;

    virtual std::vector<OrderPtr> peekScheduledOrdersUntil(int untilTime) const = 0;
};

}  // namespace des
