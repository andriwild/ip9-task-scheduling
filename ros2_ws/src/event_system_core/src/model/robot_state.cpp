#include "robot_state.h"

#include <rclcpp/rclcpp.hpp>

#include "../util/log.h"
#include "robot.h"

double RobotState::getEnergyConsumption(const Robot& robot, const des::SimConfig& cfg) const {
    return robot.isDriving()
        ? cfg.energyConsumptionDrive
        : cfg.energyConsumptionBase;
}

void IdleState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot.state"), "Enter Idle");
    robot.setSpeed(robot.getDriveSpeed());
}
double IdleState::getEnergyConsumption(const Robot& robot, const des::SimConfig& cfg) const {
    if (robot.isDriving() && robot.getTargetLocation() == robot.getIdleLocation()) {
        return cfg.energyConsumptionDrive;
    }
    if (robot.getLocation() == robot.getIdleLocation()) {
        return 0.0;
    }
    return cfg.energyConsumptionBase;
}

void ChargeState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot.state"), "Enter Charge");
}
double ChargeState::getEnergyConsumption(const Robot& robot, const des::SimConfig& cfg) const {
    auto energyConsumption = cfg.energyConsumptionBase;
    if (robot.isDriving()) {
        energyConsumption = cfg.energyConsumptionDrive;
    } else if (robot.getLocation() == robot.getIdleLocation()) {
        return robot.chargingConsumption(cfg.chargingRate, cfg.energyConsumptionBase);
    }
    return energyConsumption;
}
