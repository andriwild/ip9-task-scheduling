#include "robot_state.h"

#include <rclcpp/rclcpp.hpp>

#include "../util/log.h"
#include "i_sim_context.h"
#include "robot.h"

void IdleState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot.state"), "Enter Idle");
    robot.setSpeed(robot.getDriveSpeed());
}
void IdleState::exit(Robot& robot) {
    RobotState::exit(robot);
}
double IdleState::getEnergyConsumption(const ISimContext& ctx) const {
    const auto robot = ctx.getRobot();
    if (robot->isDriving() && robot->getTargetLocation() == robot->getIdleLocation()) {
        return ctx.getConfig()->energyConsumptionDrive;
    }
    if (robot->getLocation() == robot->getIdleLocation()) {
        return 0.0;
    }
    return ctx.getConfig()->energyConsumptionBase;
}

void ChargeState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot.state"), "Enter Charge");
}
void ChargeState::exit(Robot& robot) {
    RobotState::exit(robot);
}
double ChargeState::getEnergyConsumption(const ISimContext& ctx) const {
    auto energyConsumption = ctx.getConfig()->energyConsumptionBase;
    if (ctx.getRobot()->isDriving()) {
        energyConsumption = ctx.getConfig()->energyConsumptionDrive;
    } else if (ctx.getRobot()->getLocation() == ctx.getRobot()->getIdleLocation()) {
        return ctx.getRobot()->m_bat->chargingConsumption(ctx.getConfig()->chargingRate, ctx.getConfig()->energyConsumptionBase);
    }
    return energyConsumption;
}
