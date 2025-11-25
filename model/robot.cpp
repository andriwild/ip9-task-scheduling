#include "robot.h"
#include "robot_state.h"


void Robot::changeState(RobotState* newState) {
    state->exit(*this);
    state.reset(newState);
    state->enter(*this);
}

void Robot::setSpeed(int newSpeed) {
    currentSpeed = newSpeed;
}

double Robot::getSpeed() const {
    return currentSpeed;
};

bool Robot::isBusy() {
    if(!state) return false;
    return state->isBusy();
};

bool Robot::isSearching() {
    if(!state) return false;
    return state->isSearching();
};


bool Robot::isEscorting() {
    if(!state) return false;
    return state->isEscorting();
};

std::string Robot::getLocation() const {
    return currentLocation;
};

void Robot::moveTo(std::string location) {
    currentLocation = location;
};
