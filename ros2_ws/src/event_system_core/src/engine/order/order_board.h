/*
 * Owns the current mission and the three mission channels:
 * scheduled, background and interrupt.
 * Scheduled orders live in the event queue as dispatch events,
 * so this class reads them back from there rather than storing them twice.
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
    EventQueue& m_queue;
    EventBus& m_bus;

public:
    OrderBoard(EventQueue& queue, EventBus& bus);

    void setCurrent(const OrderPtr& orderPtr);
    OrderPtr current() const;
    OrderPtr effective() const;
    void updateState(const OrderState& newState);
    void complete(ISimContext& ctx, const OrderPtr& order);

    void addScheduled(const OrderPtr& orderPtr);
    bool hasScheduled() const;
    OrderPtr nextScheduled();
    OrderPtr popScheduled();
    std::optional<int> nextScheduledDispatchTime() const;

    // reading directly from the event queue
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
