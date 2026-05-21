#pragma once

#include <memory>
#include <string>

#include "../../model/robot_state.h"
#include "../../util/types.h"

class Robot;
class ISimContext;

// CleanState — robot is executing a clean mission. Energy: drive cost while
// driving to/from the room, base cost while actively sweeping. (A dedicated
// cleaning power can be added later via a new SimConfig field.)
class CleanState final : public RobotState {
public:
    explicit CleanState() = default;
    void enter(Robot& robot) override;
    void exit(Robot& robot) override;
    des::RobotStateType getType() const override { return des::RobotStateType::MISSION; }
    std::string getName() const override { return "clean"; }
    double getEnergyConsumption(const ISimContext& ctx) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<CleanState>(*this); }
};
