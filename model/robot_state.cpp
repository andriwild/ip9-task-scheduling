#include <iostream>

#include "robot.h"
#include "robot_state.h"

bool IdleState::isBusy() const { return false; };

void IdleState::enter(Robot& robot) { 
    std::cout << "[State] enter idle " << std::endl;
    robot.setSpeed(DEFAULT_SPEED);
}
void IdleState::exit(Robot& robot) {
    std::cout << "[State] exit idle " << std::endl;
}
void IdleState::handleEvent(Robot& robot) { 
    std::cout << "[State] handle idle " << std::endl;
}

void EscortState::enter(Robot& robot) { 
    std::cout << "[State] enter escort " << std::endl;
    robot.setSpeed(ESCORT_SPEED);
}
void EscortState::exit(Robot& robot) {
    std::cout << "[State] exit escort " << std::endl;
}
void EscortState::handleEvent(Robot& robot) { 
    std::cout << "[State] handle escort " << std::endl;
}
bool EscortState::isEscorting() const { return true; };


void MoveState::enter(Robot& robot) { 
    std::cout << "[State] enter move " << std::endl;
    robot.setSpeed(DEFAULT_SPEED);
}
void MoveState::exit(Robot& robot) {
    std::cout << "[State] exit move " << std::endl;
}
void MoveState::handleEvent(Robot& robot) { 
    std::cout << "[State] handle move " << std::endl;
}


void SearchState::enter(Robot& robot) { 
    std::cout << "[State] enter search " << std::endl;
    robot.setSpeed(DEFAULT_SPEED);
}
void SearchState::exit(Robot& robot) {
    std::cout << "[State] exit search " << std::endl;
}
void SearchState::handleEvent(Robot& robot) { 
    std::cout << "[State] handle search " << std::endl;
}

bool SearchState::isSearching() const { return true; };
