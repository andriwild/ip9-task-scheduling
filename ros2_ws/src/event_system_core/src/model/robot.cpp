#include "robot.h"
#include "robot_state.h"

void Robot::changeState(std::unique_ptr<RobotState> newState) {
    m_state->exit(*this);
    m_state = std::move(newState);
    m_state->enter(*this);
}

bool Robot::isBusy() const {
    return m_state->getType() == des::RobotStateType::SEARCHING
        || m_state->getType() == des::RobotStateType::ACCOMPANY
        || m_state->getType() == des::RobotStateType::CONVERSATE;
};

void Robot::updateConfig(const des::SimConfig& config) {
    setDriveSpeed(config.robotSpeed);
    setAccompanySpeed(config.robotAccompanySpeed);
    m_dockLocation = config.dockLocation;
    m_bat->updateConfig(
        config.batteryCapacity,
        config.initialBatteryCapacity,
        config.lowBatteryThreshold
    );
};