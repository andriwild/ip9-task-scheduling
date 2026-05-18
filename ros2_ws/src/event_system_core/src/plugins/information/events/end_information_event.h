#pragma once

#include "model/event/base.h"
#include "model/event/mission_complete_event.h"
#include "model/i_sim_context.h"

class EndInformationEvent final : public IEvent {
public:
    explicit EndInformationEvent(const int time) : IEvent(time) {}

    void execute(ISimContext& ctx) override {
        ctx.updateOrderState(des::MissionState::COMPLETED);
        ctx.notifyEvent(*this);
        ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, ctx.getOrderPtr()));
        ctx.tickBT();
    }

    std::string getName() const override { return "End Information"; }
    des::EventType getType() const override { return des::EventType::INFORMATION; }
};
