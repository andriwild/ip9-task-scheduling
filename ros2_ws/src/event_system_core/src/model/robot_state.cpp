#include "robot_state.h"

#include <rclcpp/rclcpp.hpp>

#include "context.h"
#include "robot.h"

void IdleState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(robot.getLogger(), "[IDLE] Enter");
}
void IdleState::exit(Robot& robot) {
    RobotState::exit(robot);
    RCLCPP_DEBUG(robot.getLogger(), "[IDLE] Exit");
}
des::RobotStateType IdleState::getType() const {
    return des::RobotStateType::IDLE;
}
double IdleState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionBase;
};


void AccompanyState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(robot.getLogger(), "[ACCOMPANY] Enter");
    robot.setSpeed(robot.getAccompanySpeed());
}
void AccompanyState::exit(Robot& robot) {
    RobotState::exit(robot);
    RCLCPP_DEBUG(robot.getLogger(), "[ACCOMPANY] Exit");
}
des::RobotStateType AccompanyState::getType() const {
    return des::RobotStateType::ACCOMPANY;
}
double AccompanyState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionDrive;
};


void SearchState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(robot.getLogger(), "[SEARCH] Enter");
    robot.setSpeed(robot.getDriveSpeed());
}
void SearchState::exit(Robot& robot) {
    RobotState::exit(robot);
    RCLCPP_DEBUG(robot.getLogger(), "[SEARCH] Exit");
}
des::RobotStateType SearchState::getType() const {
    return des::RobotStateType::SEARCHING;
}
double SearchState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionDrive;
};


void ConversateState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(robot.getLogger(), "[CONVERSATE] Enter");
}
void ConversateState::exit(Robot& robot) {
    RobotState::exit(robot);
    RCLCPP_DEBUG(robot.getLogger(), "[CONVERSATE] Exit");
}
des::RobotStateType ConversateState::getType() const {
    return des::RobotStateType::CONVERSATE;
}

double ConversateState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionBase;
};


void ChargeState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(robot.getLogger(), "[CHARGE] Enter");
}
void ChargeState::exit(Robot& robot) {
    RobotState::exit(robot);
    RCLCPP_DEBUG(robot.getLogger(), "[CHARGE] Exit");
}
des::RobotStateType ChargeState::getType() const {
    return des::RobotStateType::CHARGING;
}

double ChargeState::getEnergyConsumption(const SimulationContext& ctx) const {
    // Robot is running while charging
    return ctx.getConfig()->chargingRate + ctx.getConfig()->energyConsumptionBase;
};

