#include "robot.h"
#include "robot_state.h"

void Robot::changeState(std::unique_ptr<RobotState> newState) {
    if (m_state) { m_state->exit(*this); }
    m_state = std::move(newState);
    m_state->enter(*this);
}

bool Robot::isBusy() const {
    return m_state->getType() == des::RobotStateType::SEARCHING
        || m_state->getType() == des::RobotStateType::ACCOMPANY
        || m_state->getType() == des::RobotStateType::CONVERSATE
        || m_state->getType() == des::RobotStateType::CHARGING
        // limitation: we cannot stop driving between two locations -> driving without a job is busy
        || isDriving();
};

void Robot::updateConfig(const des::SimConfig& config) {
    RCLCPP_INFO(rclcpp::get_logger("Robot"), "Robot: Updating configuration");
    setDriveSpeed(config.robotSpeed);
    setAccompanySpeed(config.robotAccompanySpeed);
    m_dockLocation = config.dockLocation;
    m_bat->updateConfig(
        config.batteryCapacity,
        config.initialBatteryCapacity,
        config.lowBatteryThreshold,
        config.fullBatteryThreshold
    );
};