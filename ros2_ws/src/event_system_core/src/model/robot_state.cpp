#include <iostream>

#include "robot.h"
#include "robot_state.h"

bool IdleState::isBusy() const { return false; };

void IdleState::enter(Robot& robot) { 
    std::cout << "[State] enter idle " << std::endl;
    robot.setSpeed(robot.getDefaultSpeed());
}
void IdleState::exit(Robot& robot) {
    std::cout << "[State] exit idle " << std::endl;
}
void IdleState::handleEvent(Robot& robot) { 
    std::cout << "[State] handle idle " << std::endl;
}
RobotStateType IdleState::getType() const {
    return RobotStateType::IDLE;
}

void EscortState::enter(Robot& robot) { 
    std::cout << "[State] enter escort " << std::endl;
    robot.setSpeed(robot.getEscortSpeed());
}
void EscortState::exit(Robot& robot) {
    std::cout << "[State] exit escort " << std::endl;
}
void EscortState::handleEvent(Robot& robot) { 
    std::cout << "[State] handle escort " << std::endl;
}
bool EscortState::isEscorting() const { return true; };

RobotStateType EscortState::getType() const {
    return RobotStateType::ESCORTING;
}


void MoveState::enter(Robot& robot) { 
    std::cout << "[State] enter move " << std::endl;
    robot.setSpeed(robot.getDefaultSpeed());
}
void MoveState::exit(Robot& robot) {
    std::cout << "[State] exit move " << std::endl;
}
void MoveState::handleEvent(Robot& robot) { 
    std::cout << "[State] handle move " << std::endl;
}
RobotStateType MoveState::getType() const {
    return RobotStateType::MOVING;
}


void SearchState::enter(Robot& robot) { 
    std::cout << "[State] enter search " << std::endl;
    robot.setSpeed(robot.getDefaultSpeed());
}
void SearchState::exit(Robot& robot) {
    std::cout << "[State] exit search " << std::endl;
}
void SearchState::handleEvent(Robot& robot) { 
    std::cout << "[State] handle search " << std::endl;
}

bool SearchState::isSearching() const { return true; };
RobotStateType SearchState::getType() const {
    return RobotStateType::SEARCHING;
}


void ConversateState::enter(Robot& robot) { 
    std::cout << "[State] enter conversation" << std::endl;
    robot.setSpeed(robot.getDefaultSpeed());
}
void ConversateState::exit(Robot& robot) {
    std::cout << "[State] exit conversation " << std::endl;
}
void ConversateState::handleEvent(Robot& robot) { 
    std::cout << "[State] handle conversation" << std::endl;
}

bool ConversateState::isConversate() const { return true; };
RobotStateType ConversateState::getType() const {
    return RobotStateType::CONVERSATE;
}
