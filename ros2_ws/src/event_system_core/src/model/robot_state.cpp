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
RobotStateType IdleState::getType() const {
    return RobotStateType::IDLE;
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

RobotStateType AccompanyState::getType() const {
    return RobotStateType::ACCOMPANY;
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
RobotStateType MoveState::getType() const {
    return RobotStateType::MOVING;
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
RobotStateType SearchState::getType() const {
    return RobotStateType::SEARCHING;
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
RobotStateType ConversateState::getType() const {
    return RobotStateType::CONVERSATE;
}
