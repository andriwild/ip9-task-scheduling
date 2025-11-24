#pragma once

#include "robot.h"
#include "event.h"

class SimulationContext {
public:
    Robot& robot;
    EventQueue& queue;

    explicit SimulationContext(Robot& robot, EventQueue& queue):
        robot(robot),
        queue(queue)
    {}
};
