/*
 * Owns the current mission and the three mission channels:
 * scheduled, background and interrupt.
 *
 */
#pragma once

#include <optional>
#include <vector>

#include "engine/order/scheduled_order_queue.h"
#include "engine/order/background_order_pool.h"
#include "engine/order/interrupt_order_slot.h"
#include "util/types.h"

namespace des {

class ISimContext;
class EventQueue;
class EventBus;
class OrderBoard {
    OrderPtr m_current = nullptr;
    ScheduledOrderQueue m_scheduled;
    BackgroundOrderPool m_background;
    InterruptOrderSlot m_interrupt;
    OrderList m_dispatchPlan;
    EventQueue& m_queue;
    EventBus& m_bus;

public:
    OrderBoard(EventQueue& queue, EventBus& bus);

    void setCurrent(const OrderPtr& orderPtr);
    OrderPtr current() const;
    OrderPtr effective() const;
    void updateState(const OrderState& newState) const;
    void complete(ISimContext& ctx, const OrderPtr& order);

    void setDispatchPlan(const OrderList& orders);
    void addScheduled(const OrderPtr& orderPtr);
    bool hasScheduled() const;
    OrderPtr nextScheduled() const;
    OrderPtr popScheduled();
    std::optional<int> nextScheduledDispatchTime() const;
    OrderPtr peekNextScheduledOrder() const;
    std::vector<OrderPtr> peekScheduledOrdersUntil(int untilTime) const;

    void addBackground(const OrderPtr& orderPtr);
    bool hasBackground() const;
    OrderPtr acceptFeasibleBackground(ISimContext& ctx);

    bool pushInterrupt(ISimContext& ctx, const OrderPtr& order);
    void popInterrupt(ISimContext& ctx, const OrderPtr& completedOrder);
    bool hasActiveInterrupt() const;

    void reset();
};

}  // namespace des
