#include "robot.h"
#include "robot_state.h"


void Robot::changeState(RobotState* newState) {
    state->exit(*this);
    state.reset(newState);
    state->enter(*this);
}

void Robot::setSpeed(int newSpeed) {
    speed = newSpeed;
}

bool Robot::isBusy() {
    if(!state) return false;
    return state->isBusy();
};
