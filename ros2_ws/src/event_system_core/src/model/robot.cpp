#include "robot.h"

#include "robot_state.h"

void Robot::changeState(std::unique_ptr<RobotState> newState) {
    m_state->exit(*this);
    m_state = std::move(newState);
    m_state->enter(*this);
}

bool Robot::isBusy() {
    if (!m_state) {
        return false;
    }
    return m_state->isBusy();
};

bool Robot::isSearching() {
    if (!m_state) {
        return false;
    }
    return m_state->isSearching();
};

bool Robot::isAccompany() {
    if (!m_state) {
        return false;
    }
    return m_state->isAccompany();
};

bool Robot::isConversate() {
    if (!m_state) {
        return false;
    }
    return m_state->isConversate();
};

std::string Robot::getLocation() const {
    return m_currentLocation;
};

void Robot::setLocation(std::string location, double distance) {
    m_currentLocation = location;
    m_bat->discharge(m_currentSpeed, distance);
};
