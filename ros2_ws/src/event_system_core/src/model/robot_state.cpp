#include "robot_state.h"

#include <rclcpp/rclcpp.hpp>

#include "robot.h"

bool IdleState::isBusy() const { return false; };

void IdleState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] enter idle ");
    robot.setSpeed(robot.getSpeed());
}
void IdleState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] exit idle ");
}
void IdleState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] handle idle ");
}
des::RobotStateType IdleState::getType() const {
    return des::RobotStateType::IDLE;
}

void AccompanyState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] enter accompany ");
    robot.setSpeed(robot.getAccompanySpeed());
}
void AccompanyState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] exit accompany ");
}
void AccompanyState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] handle accompany ");
}
bool AccompanyState::isAccompany() const { return true; };

des::RobotStateType AccompanyState::getType() const {
    return des::RobotStateType::ACCOMPANY;
}

void MoveState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] enter move ");
    robot.setSpeed(robot.getSpeed());
}
void MoveState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] exit move ");
}
void MoveState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] handle move ");
}
des::RobotStateType MoveState::getType() const {
    return des::RobotStateType::MOVING;
}

void SearchState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] enter search ");
    robot.setSpeed(robot.getSpeed());
}
void SearchState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] exit search ");
}
void SearchState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] handle search ");
}

bool SearchState::isSearching() const { return true; };
des::RobotStateType SearchState::getType() const {
    return des::RobotStateType::SEARCHING;
}

void ConversateState::enter(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] enter conversation");
    robot.setSpeed(robot.getSpeed());
}
void ConversateState::exit(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] exit conversation ");
}
void ConversateState::handleEvent(Robot& robot) {
    RCLCPP_DEBUG(robot.getLogger(), "[State] handle conversation");
}

bool ConversateState::isConversate() const { return true; };
des::RobotStateType ConversateState::getType() const {
    return des::RobotStateType::CONVERSATE;
}
