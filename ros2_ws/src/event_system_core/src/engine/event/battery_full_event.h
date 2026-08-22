#pragma once

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"

namespace des {

class BatteryFullEvent final : public IEvent {
public:
    explicit BatteryFullEvent(const int time) : IEvent(time) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<BatteryFullEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        if (ctx.getRobot()->getStateType() != RobotStateType::CHARGING) {
            ctx.getRobot()->m_batteryFullEventScheduled = false;
            return;
        }
        ctx.getRobot()->completeCharge();
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.getRobot()->m_batteryFullEventScheduled = false;
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Battery Full"; }
    EventType getType() const override { return EventType::BATTERY_FULL; }
};

}  // namespace des
