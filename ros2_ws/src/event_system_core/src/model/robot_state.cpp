#include "robot_state.h"

#include <rclcpp/rclcpp.hpp>

#include "../util/log.h"
#include "i_sim_context.h"
#include "robot.h"

double RobotState::getEnergyConsumption(const ISimContext& ctx) const {
    return ctx.getRobot()->isDriving()
        ? ctx.getConfig()->energyConsumptionDrive
        : ctx.getConfig()->energyConsumptionBase;
}

void IdleState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot.state"), "Enter Idle");
    robot.setSpeed(robot.getDriveSpeed());
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
double ChargeState::getEnergyConsumption(const ISimContext& ctx) const {
    auto energyConsumption = ctx.getConfig()->energyConsumptionBase;
    if (ctx.getRobot()->isDriving()) {
        energyConsumption = ctx.getConfig()->energyConsumptionDrive;
    } else if (ctx.getRobot()->getLocation() == ctx.getRobot()->getIdleLocation()) {
        return ctx.getRobot()->chargingConsumption(ctx.getConfig()->chargingRate, ctx.getConfig()->energyConsumptionBase);
    }
    return energyConsumption;
}
