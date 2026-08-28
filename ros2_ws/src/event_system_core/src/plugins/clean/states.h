#pragma once

#include <string>

#include "../../model/robot_state.h"
#include "../../util/types.h"
#include "model/sim_config.h"

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
};

}  // namespace des
