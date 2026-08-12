#pragma once

#include <format>

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"

namespace des {

class IEvent;
class Robot;
class Scheduler;
class MissionDispatchEvent final : public IEvent {
public:
    OrderPtr orderPtr;
    explicit MissionDispatchEvent(const int time, OrderPtr order)
        : IEvent(time)
        , orderPtr(order)
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<MissionDispatchEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.publishMission(this->orderPtr, time);
        ctx.addScheduledOrder(this->orderPtr);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Mission {} Dispatch", orderPtr->id);
    }
    EventType getType() const override { return EventType::MISSION_DISPATCH; }
    int getMissionId() const override { return orderPtr ? orderPtr->id : -1; }
    OrderPtr getOrder() const override {
        return orderPtr;
    }
};

}  // namespace des
