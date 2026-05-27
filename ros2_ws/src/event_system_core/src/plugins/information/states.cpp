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
    return ctx.getConfig()->energyConsumptionBase;
}
