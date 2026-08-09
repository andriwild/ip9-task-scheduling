#pragma once

#include <memory>
#include <string>

#include "../../model/robot_state.h"
#include "../../util/types.h"

namespace des {

class Robot;
class ISimContext;

class CleanState final : public RobotState {
public:
    explicit CleanState() = default;
    void enter(Robot& robot) override;
    RobotStateType getType() const override { return RobotStateType::MISSION; }
    std::string getName() const override { return "clean"; }
    double getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<CleanState>(*this); }
};

}  // namespace des
