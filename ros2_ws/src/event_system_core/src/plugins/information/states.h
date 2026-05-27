#pragma once

#include <memory>
#include <string>

#include "../../model/robot_state.h"
#include "../../util/types.h"

class Robot;
class ISimContext;

class InformationState final : public RobotState {
public:
    explicit InformationState() = default;
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override { return des::RobotStateType::MISSION; }
    std::string getName() const override { return "information"; }
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<InformationState>(*this); }
};
