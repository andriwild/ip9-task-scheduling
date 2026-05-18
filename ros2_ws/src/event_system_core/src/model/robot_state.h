#pragma once

#include <memory>
#include <vector>
#include <string>
#include <tgmath.h>

#include "../util/types.h"

class Robot;
class ISimContext;

class RobotState {
    des::Result m_result = des::Result::SUCCESS;

public:
    virtual ~RobotState() = default;
    virtual void enter(Robot& robot) { m_result = des::Result::RUNNING; };
    virtual void exit(Robot& robot) { m_result = des::Result::SUCCESS; } ;
    virtual des::RobotStateType getType() const = 0;
    virtual double getEnergyConsumption(const ISimContext& ctx) const = 0;
    virtual std::unique_ptr<RobotState> clone() const = 0;

    des::Result getResult() const { return m_result; };
    void setResult(const des::Result result) { m_result = result; };
};

class IdleState final : public  RobotState {
public:
    explicit IdleState() : RobotState() {}
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<IdleState>(*this); }
};

class ReturningState final : public  RobotState {
public:
    explicit ReturningState() : RobotState() {}
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<ReturningState>(*this); }
};

class AccompanyState final : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<AccompanyState>(*this); }
};

class SearchState final : public  RobotState {
public:
    std::vector<std::string> locations;
    explicit SearchState(const std::vector<std::string> &locations): locations(locations) {}
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<SearchState>(*this); }
};

class ConversateState final: public  RobotState {
public:
    enum class Type {
        FOUND_PERSON,
        DROP_OFF
    };

    const Type conversationType;

    ConversateState(const Type type = Type::FOUND_PERSON) : conversationType(type) {}

    void enter(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<ConversateState>(*this); }
};

class ChargeState final : public  RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override;
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<ChargeState>(*this); }
};
