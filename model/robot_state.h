#pragma once

class Robot;

enum class RobotStateType {
    IDLE,
    MOVING,
    ESCORTING,
    CHARGING,
};

class RobotState {
public:
    virtual ~RobotState() = default;
    virtual bool isBusy() const { return true; };
    virtual void enter(Robot& robot) = 0;
    virtual void handleEvent(Robot& robot) = 0;
    virtual void exit(Robot& robot) = 0;
};

class IdleState : public  RobotState {
public:
    bool isBusy() const override;
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
};

class MoveState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
};

class EscortState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
};
