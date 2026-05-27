#include "states.h"

#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "../../model/i_sim_context.h"
#include "../../model/robot.h"
#include "accompany_plugin.h"

void SearchState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.state"), "Enter Search");
    robot.setSpeed(robot.getDriveSpeed());
}
void SearchState::exit(Robot& robot) {
    RobotState::exit(robot);
}
double SearchState::getEnergyConsumption(const ISimContext& ctx) const {
    return ctx.getRobot()->isDriving()
        ? ctx.getConfig()->energyConsumptionDrive
        : ctx.getConfig()->energyConsumptionBase;
}

void AccompanyState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.state"), "Enter Accompany");
    robot.setSpeed(accompanyConfig().accompanySpeed);
}
void AccompanyState::exit(Robot& robot) {
    RobotState::exit(robot);
}
double AccompanyState::getEnergyConsumption(const ISimContext& ctx) const {
    return ctx.getRobot()->isDriving()
        ? ctx.getConfig()->energyConsumptionDrive
        : ctx.getConfig()->energyConsumptionBase;
}

void ConversateState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.state"), "Enter Conversate");
    robot.setSpeed(robot.getDriveSpeed());
}
double ConversateState::getEnergyConsumption(const ISimContext& ctx) const {
    return ctx.getConfig()->energyConsumptionBase;
}
