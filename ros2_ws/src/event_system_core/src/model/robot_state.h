#pragma once

#include <vector>
#include <string>
#include "../util/types.h"

class Robot;
class SimulationContext;

class RobotState {
    des::Result m_result;

public:
    virtual ~RobotState() = default;
    virtual void enter(Robot& robot) { m_result = des::Result::RUNNING; };
    virtual void exit(Robot& robot) { m_result = des::Result::SUCCESS; } ;
    virtual des::RobotStateType getType() const = 0;
    virtual double getEnergyConsumption(const SimulationContext& ctx) const = 0;

    des::Result getResult() const { return m_result; };
    void setResult(des::Result result) { m_result = result; };
};

class IdleState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const SimulationContext& ctx) const override;

};

class AccompanyState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const SimulationContext& ctx) const override;
};

class SearchState : public  RobotState {
public:
    std::vector<std::string> locations;
    SearchState(std::vector<std::string> locations): locations(locations) {}
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const SimulationContext& ctx) const override;
};

class ConversateState : public  RobotState {
public:
    enum class Type {
        FOUND_PERSON,
        DROP_OFF
    };

    const Type conversationType;

    ConversateState(Type type = Type::FOUND_PERSON) : conversationType(type) {}
    
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const SimulationContext& ctx) const override;
};

class ChargeState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const SimulationContext& ctx) const override;
};
