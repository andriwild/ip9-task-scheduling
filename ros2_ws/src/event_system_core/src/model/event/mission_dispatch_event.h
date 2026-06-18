#pragma once

#include <format>

#include "base.h"
#include "../i_sim_context.h"

class IEvent;
class Robot;
class Scheduler;

class MissionDispatchEvent final : public IEvent {
public:
    des::OrderPtr orderPtr;
    explicit MissionDispatchEvent(const int time, des::OrderPtr order)
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
        ctx.addScheduledMission(this->orderPtr);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Mission {} Dispatch", orderPtr->id);
    }
    des::EventType getType() const override { return des::EventType::MISSION_DISPATCH; }
    int getMissionId() const override { return orderPtr ? orderPtr->id : -1; }
};
