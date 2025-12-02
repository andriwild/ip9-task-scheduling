#pragma once

#include <vector>
#include <string>

class Robot;

enum class RobotStateType {
    IDLE,
    MOVING,
    ESCORTING,
    SEARCHING,
    CHARGING,
};


class RobotState {
public:
    virtual ~RobotState() = default;
    virtual bool isBusy() const { return true; };
    virtual bool isSearching() const { return false; };
    virtual bool isEscorting() const { return false; };
    virtual void enter(Robot& robot) = 0;
    virtual void handleEvent(Robot& robot) = 0;
    virtual void exit(Robot& robot) = 0;
    virtual RobotStateType getType() const = 0;
};

class IdleState : public  RobotState {
public:
    bool isBusy() const override;
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
    RobotStateType getType() const override;
};

class MoveState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
    RobotStateType getType() const override;
};

class EscortState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
    bool isEscorting() const override;
    RobotStateType getType() const override;
};

class SearchState : public  RobotState {
public:
    std::vector<std::string> locations;
    SearchState(std::vector<std::string> locations):
        locations(locations)
    {}
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
    bool isSearching() const override;
    RobotStateType getType() const override;
};
