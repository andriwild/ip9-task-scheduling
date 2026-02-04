#include "robot.h"

#include "robot_state.h"

void Robot::changeState(std::unique_ptr<RobotState> newState) {
    m_state->exit(*this);
    m_state = std::move(newState);
    m_state->enter(*this);
}

std::string Robot::getLocation() const {
    return m_currentLocation;
};

void Robot::setLocation(std::string location, double distance) {
    m_currentLocation = location;
    m_bat->updateBalance(m_currentSpeed, distance);
};

bool Robot::isBusy() const {
    return m_state->getType() == des::RobotStateType::IDLE 
        || m_state->getType() == des::RobotStateType::MOVING;
};
