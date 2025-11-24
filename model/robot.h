#pragma once

#include <memory>

#include "robot_state.h"

class Robot {
    std::unique_ptr<RobotState> state;
    int speed;

public:
    Robot() {
        state = std::make_unique<IdleState>();
    }
    void changeState(RobotState* newState);
    void setSpeed(int newSpeed);
    bool isBusy();
};
