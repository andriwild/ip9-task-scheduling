#include "robot.h"
#include "../util/log.h"
#include "robot_state.h"

void Robot::changeState(std::unique_ptr<RobotState> newState) {
    if (m_state) { m_state->exit(*this); }
    m_state = std::move(newState);
    m_state->enter(*this);
}

bool Robot::isBusy() const {
    const auto type = m_state->getType();
    return type != des::RobotStateType::IDLE || isDriving();
}

void Robot::updateConfig(const des::SimConfig& config) {
    DES_LOG_INFO(rclcpp::get_logger("des.robot"), "Robot: Updating configuration");
    setDriveSpeed(config.robotSpeed);
    m_dockLocation = config.dockLocation;
    m_bat->updateConfig(
        config.batteryCapacity,
        config.initialBatteryCapacity,
        config.lowBatteryThreshold,
        config.fullBatteryThreshold
    );
};
