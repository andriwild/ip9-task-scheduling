#include "robot_state.h"

#include <rclcpp/rclcpp.hpp>

#include "context.h"
#include "robot.h"

void IdleState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[IDLE] Enter");
    robot.setSpeed(0);
}
void IdleState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[IDLE] Exit");
}
void IdleState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[IDLE] Handle Event");
}
des::RobotStateType IdleState::getType() const {
    return des::RobotStateType::IDLE;
}

double IdleState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionBase;
};


void AccompanyState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[ACCOMPANY] Enter");
    robot.setSpeed(robot.getAccompanySpeed());
}
void AccompanyState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[ACCOMPANY] Exit");
}
void AccompanyState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[ACCOMPANY] Hanle Event");
}
des::RobotStateType AccompanyState::getType() const {
    return des::RobotStateType::ACCOMPANY;
}

double AccompanyState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionDrive;
};


void MoveState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[MOVE] Enter");
    robot.setSpeed(robot.getDriveSpeed());
}
void MoveState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[MOVE] Exit");
}
void MoveState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[MOVE] Handle Event");
}
des::RobotStateType MoveState::getType() const {
    return des::RobotStateType::MOVING;
}

double MoveState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionDrive;
};


void SearchState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[SEARCH] Enter");
    robot.setSpeed(robot.getDriveSpeed());
}
void SearchState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[SEARCH] Exit");
}
void SearchState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[SEARCH] Handle Event");
}

des::RobotStateType SearchState::getType() const {
    return des::RobotStateType::SEARCHING;
}

double SearchState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionDrive;
};


void ConversateState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[CONVERSATE] Enter");
    robot.setSpeed(0);
}
void ConversateState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[CONVERSATE] Exit");
}
void ConversateState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[CONVERSATE] Handle Event");
}

des::RobotStateType ConversateState::getType() const {
    return des::RobotStateType::CONVERSATE;
}

double ConversateState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionBase;
};


void ChargeState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[CHARGE] Enter");
    robot.setSpeed(0);
}
void ChargeState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[CHARGE] Exit");
}
void ChargeState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[CHARGE] Handle Event");
}

des::RobotStateType ChargeState::getType() const {
    return des::RobotStateType::CHARGING;
}

double ChargeState::getEnergyConsumption(const SimulationContext& ctx) const {
    // Robot is running while charging
    return ctx.getConfig()->chargingRate + ctx.getConfig()->energyConsumptionBase;
};

