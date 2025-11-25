#pragma once

#include <memory>

#include "robot_state.h"

class Robot {
    std::unique_ptr<RobotState> state;
    int currentSpeed = 0;
    std::string currentLocation;

public:
    Robot() {
        state = std::make_unique<IdleState>();
    }

    std::string getLocation() const;
    void moveTo(std::string location);

    void changeState(RobotState* newState);
    void setSpeed(int newSpeed);
    double getSpeed() const;
    bool isBusy();
    bool isSearching();
    bool isEscorting();
};
