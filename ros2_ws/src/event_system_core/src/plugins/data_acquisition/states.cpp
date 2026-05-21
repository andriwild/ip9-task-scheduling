#include "states.h"

#include <rclcpp/rclcpp.hpp>

#include "../../model/i_sim_context.h"
#include "../../model/robot.h"

void AcquireState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(rclcpp::get_logger("State"), "Enter Acquire");
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
