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
    virtual void exit(Robot& robot) = 0;
    virtual des::RobotStateType getType() const = 0;
    virtual double getEnergyConsumption(const SimulationContext& ctx) const = 0;
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
    bool m_successful = false;

    ConversateState(Type type = Type::FOUND_PERSON) : conversationType(type) {}
    
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const SimulationContext& ctx) const override;
    void complete(bool successful) { m_successful = successful; } 
    bool isSuccessful() const { return m_successful; }
};

class ChargeState : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const SimulationContext& ctx) const override;
};
