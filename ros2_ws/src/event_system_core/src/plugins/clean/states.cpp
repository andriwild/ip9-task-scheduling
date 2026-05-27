#include "states.h"

#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "../../model/i_sim_context.h"
#include "../../model/robot.h"

void CleanState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.clean.state"), "Enter Clean");
    robot.setSpeed(robot.getDriveSpeed());
}
void CleanState::exit(Robot& robot) {
    RobotState::exit(robot);
}
double CleanState::getEnergyConsumption(const ISimContext& ctx) const {
    return ctx.getRobot()->isDriving()
        ? ctx.getConfig()->energyConsumptionDrive
        : ctx.getConfig()->energyConsumptionBase;
}
