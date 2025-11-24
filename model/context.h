#pragma once

#include <cassert>
#include <vector>

#include "robot.h"
#include "event.h"
#include "../observer/observer.h"


class SimulationContext {
    int currentTime = 0;
    std::vector<std::shared_ptr<IObserver>> observers;

public:
    Robot& robot;
    EventQueue& queue;

    explicit SimulationContext(Robot& robot, EventQueue& queue):
        robot(robot),
        queue(queue)
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
