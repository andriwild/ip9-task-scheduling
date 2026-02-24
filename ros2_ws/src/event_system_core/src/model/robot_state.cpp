#include "robot_state.h"

#include <rclcpp/rclcpp.hpp>

#include "context.h"
#include "robot.h"

void IdleState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(rclcpp::get_logger("State"), "Enter Idle");
}
void IdleState::exit(Robot& robot) {
    RobotState::exit(robot);
}
des::RobotStateType IdleState::getType() const {
    return des::RobotStateType::IDLE;
}
double IdleState::getEnergyConsumption(const SimulationContext& ctx) const {
    auto energyConsumption = ctx.getConfig()->energyConsumptionBase;
    if (ctx.m_robot->isDriving()) {
        energyConsumption = ctx.getConfig()->energyConsumptionDrive;
    } else if (ctx.m_robot->getLocation() == ctx.m_robot->getIdleLocation()) {
        energyConsumption -= ctx.getConfig()->chargingRate;
    }
    return energyConsumption;
}


void AccompanyState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(rclcpp::get_logger("State"), "Enter Accompany");
    robot.setSpeed(robot.getAccompanySpeed());
}
void AccompanyState::exit(Robot& robot) {
    RobotState::exit(robot);
}
des::RobotStateType AccompanyState::getType() const {
    return des::RobotStateType::ACCOMPANY;
}
double AccompanyState::getEnergyConsumption(const SimulationContext& ctx) const {
    if (ctx.m_robot->isDriving()) {
        return ctx.getConfig()->energyConsumptionDrive;
    }
    return ctx.getConfig()->energyConsumptionBase;
}


void SearchState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(rclcpp::get_logger("State"), "Enter Search");
    robot.setSpeed(robot.getDriveSpeed());
}
void SearchState::exit(Robot& robot) {
    RobotState::exit(robot);
}
des::RobotStateType SearchState::getType() const {
    return des::RobotStateType::SEARCHING;
}
double SearchState::getEnergyConsumption(const SimulationContext& ctx) const {
    if (ctx.m_robot->isDriving()) {
        return ctx.getConfig()->energyConsumptionDrive;
    }
    return ctx.getConfig()->energyConsumptionBase;
}


void ConversateState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(rclcpp::get_logger("State"), "Enter Conversate");
    robot.setSpeed(robot.getDriveSpeed());
}
des::RobotStateType ConversateState::getType() const {
    return des::RobotStateType::CONVERSATE;
}
double ConversateState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionBase;
}

void ChargeState::enter(Robot& robot) {
    RobotState::enter(robot);
    RCLCPP_DEBUG(rclcpp::get_logger("State"), "Enter Charge");
}
void ChargeState::exit(Robot& robot) {
    RobotState::exit(robot);
}

des::RobotStateType ChargeState::getType() const {
    return des::RobotStateType::CHARGING;
}
double ChargeState::getEnergyConsumption(const SimulationContext& ctx) const {
    auto energyConsumption = ctx.getConfig()->energyConsumptionBase;
    if (ctx.m_robot->isDriving()) {
        energyConsumption = ctx.getConfig()->energyConsumptionDrive;
    } else if (ctx.m_robot->getLocation() == ctx.m_robot->getIdleLocation()) {
        energyConsumption -= ctx.getConfig()->chargingRate;
    }
    return energyConsumption;
}

