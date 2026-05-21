#include "robot.h"
#include "robot_state.h"

void Robot::changeState(std::unique_ptr<RobotState> newState) {
    if (m_state) { m_state->exit(*this); }
    m_state = std::move(newState);
    m_state->enter(*this);
}

bool Robot::isBusy() const {
    const auto type = m_state->getType();
    // Anything that isn't IDLE is "busy" in the sense that it shouldn't be
    // interrupted by another idle-tier action. Also driving without a state
    // counts as busy (no way to stop mid-segment).
    return type != des::RobotStateType::IDLE || isDriving();
}

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
