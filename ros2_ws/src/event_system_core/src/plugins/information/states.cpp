#include "states.h"

#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "../../model/i_sim_context.h"
#include "../../model/robot.h"

void InformationState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.information.state"), "Enter Information");
}

void InformationState::exit(Robot& robot) {
    RobotState::exit(robot);
}

double InformationState::getEnergyConsumption(const ISimContext& ctx) const {
    const auto robot = ctx.getRobot();
    // At the dock the robot is plugged in, so an information interrupt keeps charging instead of draining.
    if (!robot->isDriving() && robot->getLocation() == robot->getIdleLocation()) {
        return robot->m_bat->chargingConsumption(ctx.getConfig()->chargingRate, ctx.getConfig()->energyConsumptionBase);
    }
    return ctx.getConfig()->energyConsumptionBase;
}
