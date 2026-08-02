#pragma once

#include <memory>
#include <string>

#include "../../model/robot_state.h"
#include "../../util/types.h"

class Robot;
class ISimContext;

class CleanState final : public RobotState {
public:
    explicit CleanState() = default;
    void enter(Robot& robot) override;
    des::RobotStateType getType() const override { return des::RobotStateType::MISSION; }
    std::string getName() const override { return "clean"; }
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<CleanState>(*this); }
};
