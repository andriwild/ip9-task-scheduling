#include "robot.h"

#include "robot_state.h"


void Robot::changeState(std::unique_ptr<RobotState> newState) {
    m_state->exit(*this);
    m_state = std::move(newState);
    m_state->enter(*this);
}

void Robot::setSpeed(double newSpeed) {
    m_currentSpeed = newSpeed;
}

double Robot::getSpeed() const {
    return m_currentSpeed;
};

bool Robot::isBusy() {
    if(!m_state) return false;
    return m_state->isBusy();
};

bool Robot::isSearching() {
    if(!m_state) return false;
    return m_state->isSearching();
};


bool Robot::isEscorting() {
    if(!m_state) return false;
    return m_state->isEscorting();
};


bool Robot::isConversate() {
    if(!m_state) return false;
    return m_state->isConversate();
};

std::string Robot::getLocation() const {
    return m_currentLocation;
};

void Robot::setLocation(std::string location) {
    m_currentLocation = location;
};
