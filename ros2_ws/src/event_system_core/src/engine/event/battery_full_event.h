#pragma once

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"

namespace des {

class BatteryFullEvent final : public IEvent {
    int m_epoch;
public:
    explicit BatteryFullEvent(const int time, const int epoch) : IEvent(time), m_epoch(epoch) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<BatteryFullEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    bool isStale(const ISimContext& ctx) const override {
        return ctx.getRobot()->chargeEpoch() != m_epoch;
    }

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->completeCharge();
        ctx.getRobot()->endChargePhase();
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Battery Full"; }
    EventType getType() const override { return EventType::BATTERY_FULL; }
};

}  // namespace des
