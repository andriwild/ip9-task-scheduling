#pragma once

#include "base.h"
#include "stop_drive_event.h"
#include "../robot_state.h"

class SimulationStartEvent final : public IEvent {
public:
    explicit SimulationStartEvent(const int time) : IEvent(time) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<SimulationStartEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.notifyEvent(*this);
        ctx.pushEvent(std::make_shared<StopDriveEvent>(time, ctx.getRobot()->getLocation(), 0));
    }

    std::string getName() const override { return "Simulation Start"; }
    des::EventType getType() const override { return des::EventType::SIMULATION_START; }
};
