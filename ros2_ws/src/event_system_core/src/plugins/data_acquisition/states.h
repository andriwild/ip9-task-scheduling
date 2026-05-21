#pragma once

#include <memory>
#include <string>

#include "../../model/robot_state.h"
#include "../../util/types.h"

class Robot;
class ISimContext;

// AcquireState — robot is executing a data-acquisition mission. Most of the
// mission is driving to/from the room with a brief scan in between, so the
// energy model is drive-cost when moving and base-cost when capturing.
class AcquireState final : public RobotState {
public:
    explicit AcquireState() = default;
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override { return des::RobotStateType::MISSION; }
    std::string getName() const override { return "acquire"; }
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<AcquireState>(*this); }
};
