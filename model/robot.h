#pragma once

#include <memory>

#include "robot_state.h"

class Robot {
    std::string idleLocation = "Dock";
    std::unique_ptr<RobotState> state;
    double currentSpeed = 0;
    std::string currentLocation;

public:
    Robot() {
        state = std::make_unique<IdleState>();
    }

    std::string getLocation() const;
    void setLocation(std::string location);

    void changeState(RobotState* newState);
    RobotState* getState() { return state.get(); };

    void setSpeed(double newSpeed);
    double getSpeed() const;

    std::string getIdleLocation() const {
        return idleLocation;
    }

    bool isBusy();
    bool isSearching();
    bool isEscorting();
};
