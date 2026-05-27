#include "states.h"

#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "../../model/i_sim_context.h"
#include "../../model/robot.h"

void AcquireState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.data_acquisition.state"), "Enter Acquire");
    robot.setSpeed(robot.getDriveSpeed());
}
void AcquireState::exit(Robot& robot) {
    RobotState::exit(robot);
}
double AcquireState::getEnergyConsumption(const ISimContext& ctx) const {
    return ctx.getRobot()->isDriving()
        ? ctx.getConfig()->energyConsumptionDrive
        : ctx.getConfig()->energyConsumptionBase;
}
