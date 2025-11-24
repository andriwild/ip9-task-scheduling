#include <memory>

#include "robot_state.h"
#include "event.h"
#include "context.h"


void SimulationStartEvent::execute(SimulationContext& ctx) {
    if(ctx.robot.isBusy()) {
        ctx.robot.changeState(new IdleState());
    }
    ctx.notifyLog("Simulation start");
}

void SimulationEndEvent::execute(SimulationContext& ctx) {
    if(ctx.robot.isBusy()) {
        ctx.robot.changeState(new IdleState());
    }
    ctx.notifyLog("Simulation end");
}

void ArrivedEvent::execute(SimulationContext& ctx) {
    if(ctx.robot.isBusy()) {
        ctx.robot.changeState(new IdleState());
        ctx.notifyLog("Arrived");
    }
}

void MissionDispatchEvent::execute(SimulationContext& ctx) {
    if(!ctx.robot.isBusy()) {
        ctx.robot.changeState(new MoveState());
        ctx.queue.push(std::make_shared<ArrivedEvent>(this->time + 100));
        ctx.notifyLog("Mission Dispatch");
    }
}

void MissionCompleteEvent::execute(SimulationContext& ctx) {
    if(ctx.robot.isBusy()) {
        ctx.robot.changeState(new IdleState());
        ctx.notifyLog("Mission Complete");
    }
}
