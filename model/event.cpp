#include <memory>

#include "robot_state.h"
#include "event.h"
#include "context.h"


void SimulationStartEvent::execute(SimulationContext& ctx) {
    if(ctx.robot.isBusy()) {
        ctx.robot.changeState(new IdleState());
    }
}

void SimulationEndEvent::execute(SimulationContext& ctx) {
    if(ctx.robot.isBusy()) {
        ctx.robot.changeState(new IdleState());
    }
}

void ArrivedEvent::execute(SimulationContext& ctx) {
    if(ctx.robot.isBusy()) {
        ctx.robot.changeState(new IdleState());
    }
}

void MissionDispatchEvent::execute(SimulationContext& ctx) {
    if(!ctx.robot.isBusy()) {
        ctx.robot.changeState(new MoveState());
        ctx.queue.push(std::make_shared<ArrivedEvent>(this->time + 100));
    }
}

void MissionCompleteEvent::execute(SimulationContext& ctx) {
    if(ctx.robot.isBusy()) {
        ctx.robot.changeState(new IdleState());
    }
}
