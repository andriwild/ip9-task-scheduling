#pragma once

#include "base.h"
#include "mission_complete_event.h"
#include "../i_sim_context.h"
#include "../robot_state.h"

class AbortSearchEvent final : public IEvent {
public:
    explicit AbortSearchEvent(const int time) : IEvent(time) {}

    void execute(ISimContext& ctx) override {
        ctx.updateAppointmentState(des::MissionState::FAILED);
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, ctx.getAppointment()));
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Abort Search"; }
    des::EventType getType() const override { return des::EventType::ABORT_SEARCH; }
};
