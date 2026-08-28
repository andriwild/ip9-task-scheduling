#pragma once

#include <string>

#include "../../model/robot_state.h"
#include "../../util/types.h"
#include "model/sim_config.h"

namespace des {

class Robot;
class ISimContext;

class InformationState final : public RobotState {
public:
    explicit InformationState() = default;
    void enter(Robot& robot) override;
    RobotStateType getType() const override { return RobotStateType::MISSION; }
    std::string getName() const override { return "information"; }
    double getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const override;
    bool chargesAtDock() const override { return true; }
};

}  // namespace des
