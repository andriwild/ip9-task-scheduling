#include "states.h"

#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "engine/contracts/i_sim_context.h"
#include "../../model/robot.h"

void InformationState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.information.state"), "Enter Information");
}

double InformationState::getEnergyConsumption(const Robot& robot, const des::SimConfig& cfg) const {
    // At the dock the robot is plugged in, so an information interrupt keeps charging instead of draining.
    if (!robot.isDriving() && robot.getLocation() == robot.getIdleLocation()) {
        return robot.chargingConsumption(cfg.chargingRate, cfg.energyConsumptionBase);
    }
    return cfg.energyConsumptionBase;
}
