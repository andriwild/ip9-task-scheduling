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

void Robot::setLocation(std::string location) {
    m_currentLocation = location;
};

bool Robot::isBusy() const {
    return m_state->getType() == des::RobotStateType::SEARCHING
        || m_state->getType() == des::RobotStateType::ACCOMPANY
        || m_state->getType() == des::RobotStateType::CONVERSATE;
};

void Robot::updateConfig(des::SimConfig& config) {
    setDriveSpeed(config.robotSpeed);
    setAccompanytSpeed(config.robotAccompanySpeed);
    m_bat->updateConfig(
        config.batteryCapacity,
        config.initialBatteryCapacity,
        config.lowBatteryThreshold
    );
};
