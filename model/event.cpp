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

        // SimplePose start{0.5, 0.5, 0.0};
        // SimplePose goal{0.4, 0.5, 0.0};
        // PathResult result = ctx.planner->computeDistance(start, goal);

        // if (result.success) {
        //     std::cout << "2 Distance: " << result.distance << " meters" << std::endl;
        // } else {
        //     std::cout << "2 Failed to calculate path" << std::endl;
        // }

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
