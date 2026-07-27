#pragma once

#include <optional>
#include <vector>

#include "scheduled_mission_queue.h"
#include "background_mission_pool.h"
#include "interrupt_mission_slot.h"
#include "../../util/types.h"

class ISimContext;
class EventQueue;
class EventBus;

class MissionBoard {
    des::OrderPtr m_current = nullptr;
    ScheduledMissionQueue m_scheduled;
    BackgroundMissionPool m_background;
    InterruptMissionSlot m_interrupt;
    EventQueue& m_queue;
    EventBus& m_bus;

public:
    MissionBoard(EventQueue& queue, EventBus& bus);

    void setCurrent(const des::OrderPtr& orderPtr);
    des::OrderPtr current() const;
    des::OrderPtr effective() const;
    void updateState(const des::MissionState& newState);
    void complete(ISimContext& ctx, const des::OrderPtr& order);

    void addScheduled(const des::OrderPtr& orderPtr);
    bool hasScheduled() const;
    des::OrderPtr nextScheduled();
    des::OrderPtr popScheduled();
    std::optional<int> nextScheduledDispatchTime() const;
    des::OrderPtr peekNextScheduledOrder() const;
    std::vector<des::OrderPtr> peekScheduledOrdersUntil(int untilTime) const;

    void addBackground(const des::OrderPtr& orderPtr);
    bool hasBackground() const;
    des::OrderPtr acceptFeasibleBackground(ISimContext& ctx);

    bool pushInterrupt(ISimContext& ctx, const des::OrderPtr& order);
    void popInterrupt(ISimContext& ctx, const des::OrderPtr& completedOrder);
    bool hasActiveInterrupt() const;

    void reset();
};
