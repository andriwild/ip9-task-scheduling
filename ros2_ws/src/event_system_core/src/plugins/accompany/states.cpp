#include "states.h"

#include <rclcpp/rclcpp.hpp>

#include "../../model/i_sim_context.h"
#include "../../model/robot.h"
#include "accompany_plugin.h"

void SearchState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(rclcpp::get_logger("State"), "Enter Search");
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
    RCLCPP_DEBUG(rclcpp::get_logger("State"), "Enter Accompany");
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
    RCLCPP_DEBUG(rclcpp::get_logger("State"), "Enter Conversate");
    robot.setSpeed(robot.getDriveSpeed());
}
double ConversateState::getEnergyConsumption(const ISimContext& ctx) const {
    return ctx.getConfig()->energyConsumptionBase;
}
