#pragma once

#include "model/event/base.h"
#include "model/event/mission_complete_event.h"
#include "model/i_sim_context.h"
#include "model/robot_state.h"

class AbortSearchEvent final : public IEvent {
public:
    explicit AbortSearchEvent(const int time) : IEvent(time) {}

    void execute(ISimContext& ctx) override {
        ctx.updateOrderState(des::MissionState::FAILED);
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, ctx.getOrderPtr()));
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Abort Search"; }
    des::EventType getType() const override { return des::EventType::ABORT_SEARCH; }
};
