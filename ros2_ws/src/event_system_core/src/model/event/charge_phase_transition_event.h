#pragma once

#include "base.h"
#include "../i_sim_context.h"
#include "../robot.h"
#include "../robot_state.h"

class ChargePhaseTransitionEvent final : public IEvent {
public:
    explicit ChargePhaseTransitionEvent(const int time) : IEvent(time) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<ChargePhaseTransitionEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.notifyBatteryChanged();
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Charge Phase Transition"; }
    des::EventType getType() const override { return des::EventType::CHARGE_PHASE_TRANSITION; }
    std::string getColor() const override { return "#f0a000"; }
};
