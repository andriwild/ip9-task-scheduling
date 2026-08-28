#pragma once

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"

namespace des {

class ChargePhaseTransitionEvent final : public IEvent {
    int m_epoch;
public:
    explicit ChargePhaseTransitionEvent(const int time, const int epoch) : IEvent(time), m_epoch(epoch) {}

    [[nodiscard]] std::shared_ptr<IEvent> withTime(const int newTime) const override {
        auto copy = std::make_shared<ChargePhaseTransitionEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    [[nodiscard]] bool isStale(const ISimContext& ctx) const override {
        return ctx.getRobot()->chargeEpoch() != m_epoch;
    }

    void execute(ISimContext& ctx) override {
        ctx.notifyRobotStateChanged();
        ctx.notifyEvent(*this);
    }

    [[nodiscard]] std::string getName() const override { return "Charge Phase Transition"; }
    [[nodiscard]] EventType getType() const override { return EventType::CHARGE_PHASE_TRANSITION; }
    [[nodiscard]] std::string getColor() const override { return "#f0a000"; }
};

}  // namespace des
