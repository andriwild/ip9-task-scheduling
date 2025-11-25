#pragma once

#include <cassert>
#include <vector>

#include "robot.h"
#include "event.h"
#include "../observer/observer.h"
#include "../sim/ros/path_node.h"


class SimulationContext {
    int currentTime = 0;
    std::vector<std::shared_ptr<IObserver>> observers;

public:
    Robot& robot;
    EventQueue& queue;
    std::shared_ptr<PathPlannerNode> planner;

    explicit SimulationContext(
        Robot& robot, 
        EventQueue& queue,
        std::shared_ptr<PathPlannerNode> planner
    ):
        robot(robot),
        queue(queue),
        planner(planner)
    {}

    void setTime(int newTime) {
        assert(newTime > currentTime);
        currentTime = newTime;
    }

    void addObserver(std::shared_ptr<IObserver> observer) {
        observers.emplace_back(observer);
    }

    void notifyLog(const std::string& msg) {
        for(auto obs: observers){
            obs->onLog(currentTime, msg);
        }
    }
};
