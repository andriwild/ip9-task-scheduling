/*
 * State Pattern for the robot.
 * Each state knows its own power usage, the plugins add further states.
 */
#pragma once

#include <memory>
#include <string>
#include <tgmath.h>

#include "../util/types.h"

namespace des {

struct SimConfig;

class Robot;

class RobotState {
    Result m_result = Result::SUCCESS;

public:
    virtual ~RobotState() = default;
    virtual void enter(Robot& robot);
    virtual void exit(Robot& robot);
    virtual RobotStateType getType() const = 0;
    virtual std::string getName() const = 0;
    virtual double getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const;
    virtual bool chargesAtDock() const { return false; }
    virtual std::unique_ptr<RobotState> clone() const = 0;

    Result getResult() const { return m_result; };
    void setResult(const Result result) { m_result = result; };
};

class IdleState final : public RobotState {
public:
    explicit IdleState() : RobotState() {}
    void enter(Robot& robot) override;
    RobotStateType getType() const override { return RobotStateType::IDLE; }
    std::string getName() const override { return "idle"; }
    double getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<IdleState>(*this); }
};

class ConversationState final : public RobotState {
public:
    const ConversationKind kind;
    explicit ConversationState(const ConversationKind kind) : kind(kind) {}

    void enter(Robot& robot) override;
    RobotStateType getType() const override { return RobotStateType::MISSION; }
    std::string getName() const override { return "conversate"; }
    double getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<ConversationState>(*this); }
};

class ChargeState final : public RobotState {
public:
    void enter(Robot& robot) override;
    RobotStateType getType() const override { return RobotStateType::CHARGING; }
    std::string getName() const override { return "charging"; }
    double getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const override;
    bool chargesAtDock() const override { return true; }
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<ChargeState>(*this); }
};

}  // namespace des
