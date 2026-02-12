#include "robot_state.h"

#include <rclcpp/rclcpp.hpp>

#include "context.h"
#include "robot.h"

des::RobotStateType IdleState::getType() const {
    return des::RobotStateType::IDLE;
}
double IdleState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionBase;
};


void AccompanyState::enter(Robot& robot) {
    RobotState::enter(robot);
    robot.setSpeed(robot.getAccompanySpeed());
}
void AccompanyState::exit(Robot& robot) {
    RobotState::exit(robot);
}
des::RobotStateType AccompanyState::getType() const {
    return des::RobotStateType::ACCOMPANY;
}
double AccompanyState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionDrive;
};


void SearchState::enter(Robot& robot) {
    RobotState::enter(robot);
    robot.setSpeed(robot.getDriveSpeed());
}
void SearchState::exit(Robot& robot) {
    RobotState::exit(robot);
}
des::RobotStateType SearchState::getType() const {
    return des::RobotStateType::SEARCHING;
}
double SearchState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionDrive;
};


des::RobotStateType ConversateState::getType() const {
    return des::RobotStateType::CONVERSATE;
}
double ConversateState::getEnergyConsumption(const SimulationContext& ctx) const {
    return ctx.getConfig()->energyConsumptionBase;
};


des::RobotStateType ChargeState::getType() const {
    return des::RobotStateType::CHARGING;
}
double ChargeState::getEnergyConsumption(const SimulationContext& ctx) const {
    // Robot is running while charging
    return ctx.getConfig()->chargingRate + ctx.getConfig()->energyConsumptionBase;
};

