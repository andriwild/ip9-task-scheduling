#pragma once

#include "base.h"
#include "../i_sim_context.h"
#include "../robot.h"
#include "../robot_state.h"

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
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.getRobot()->m_batteryFullEventScheduled = false;
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Battery Full"; }
    des::EventType getType() const override { return des::EventType::BATTERY_FULL; }
};
