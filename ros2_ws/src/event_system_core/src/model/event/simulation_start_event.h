#pragma once

#include "base.h"
#include "stop_drive_event.h"
#include "../i_sim_context.h"
#include "../robot.h"
#include "../robot_state.h"

class SimulationStartEvent final : public IEvent {
public:
    explicit SimulationStartEvent(const int time) : IEvent(time) {}

    void execute(ISimContext& ctx) override {
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.notifyEvent(*this);
        ctx.pushEvent(std::make_shared<StopDriveEvent>(time, ctx.getRobot()->getLocation(), 0));
    }

    std::string getName() const override { return "Simulation Start"; }
    des::EventType getType() const override { return des::EventType::SIMULATION_START; }
};
