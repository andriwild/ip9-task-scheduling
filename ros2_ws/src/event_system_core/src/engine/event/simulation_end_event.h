#pragma once

#include "engine/contracts/i_event.h"
#include "engine/event/stop_drive_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"

class SimulationEndEvent final : public IEvent {
public:
    explicit SimulationEndEvent(const int time) : IEvent(time) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<SimulationEndEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->flushRoomVisit();
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.notifyEvent(*this);
        ctx.pushEvent(std::make_shared<StopDriveEvent>(time, std::make_shared<RoomTarget>(ctx.getRobot()->getLocation()), 0));
    }

    std::string getName() const override { return "Simulation End"; }
    des::EventType getType() const override { return des::EventType::SIMULATION_END; }
};
