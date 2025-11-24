#include <iostream>

#include "robot.h"
#include "robot_state.h"

bool IdleState::isBusy() const { return false; };

void IdleState::enter(Robot& robot) { 
    std::cout << "[Robot] enter idle mode" << std::endl;
    robot.setSpeed(0);
}
void IdleState::exit(Robot& robot) {
    std::cout << "[Robot] exit idle mode" << std::endl;
}
void IdleState::handleEvent(Robot& robot) { 
    std::cout << "[Robot] handle idle mode" << std::endl;
}


void EscortState::enter(Robot& robot) { 
    std::cout << "[Robot] enter escort mode" << std::endl;
    robot.setSpeed(2);
}
void EscortState::exit(Robot& robot) {
    std::cout << "[Robot] exit escort mode" << std::endl;
}
void EscortState::handleEvent(Robot& robot) { 
    std::cout << "[Robot] handle escort mode" << std::endl;
}


void MoveState::enter(Robot& robot) { 
    std::cout << "[Robot] enter move mode" << std::endl;
    robot.setSpeed(3);
}
void MoveState::exit(Robot& robot) {
    std::cout << "[Robot] exit move mode" << std::endl;
}
void MoveState::handleEvent(Robot& robot) { 
    std::cout << "[Robot] handle move mode" << std::endl;
}
