#pragma once

#include <vector>
#include <string>
#include "../util/types.h"

class Robot;
class SimulationContext;

class RobotState {
public:
    virtual ~RobotState() = default;
    virtual void enter(Robot& robot) = 0;
    virtual void handleEvent(Robot& robot) = 0;
    virtual void exit(Robot& robot) = 0;
    virtual des::RobotStateType getType() const = 0;
    virtual double getEnergyConsumption(const SimulationContext& ctx) const = 0;
};

class IdleState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
    des::RobotStateType getType() const override;
    virtual double getEnergyConsumption(const SimulationContext& ctx) const override;
};

class MoveState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
    des::RobotStateType getType() const override;
    virtual double getEnergyConsumption(const SimulationContext& ctx) const override;
};

class AccompanyState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
    des::RobotStateType getType() const override;
    virtual double getEnergyConsumption(const SimulationContext& ctx) const override;
};

class SearchState : public  RobotState {
public:
    std::vector<std::string> locations;
    SearchState(std::vector<std::string> locations): locations(locations) {}
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
    des::RobotStateType getType() const override;
    virtual double getEnergyConsumption(const SimulationContext& ctx) const override;
};

class ConversateState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
    des::RobotStateType getType() const override;
    virtual double getEnergyConsumption(const SimulationContext& ctx) const override;
};

class ChargeState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    void handleEvent(Robot& robot) override;
    des::RobotStateType getType() const override;
    virtual double getEnergyConsumption(const SimulationContext& ctx) const override;
};
