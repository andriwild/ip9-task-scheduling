#pragma once

#include <memory>
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
    virtual std::string getName() const = 0;
    virtual double getEnergyConsumption(const ISimContext& ctx) const = 0;
    virtual std::unique_ptr<RobotState> clone() const = 0;

    des::Result getResult() const { return m_result; };
    void setResult(const des::Result result) { m_result = result; };
};

class IdleState final : public RobotState {
public:
    explicit IdleState() : RobotState() {}
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override { return des::RobotStateType::IDLE; }
    std::string getName() const override { return "idle"; }
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<IdleState>(*this); }
};

class ChargeState final : public RobotState {
public:
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override { return des::RobotStateType::CHARGING; }
    std::string getName() const override { return "charging"; }
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<ChargeState>(*this); }
};
