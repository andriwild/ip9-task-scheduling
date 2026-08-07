#pragma once

#include <memory>
#include <string>

#include "../../model/robot_state.h"
#include "../../util/types.h"

namespace des {

class Robot;
class ISimContext;

// States owned by the accompany plugin. All return MISSION as their category;
// names distinguish them inside the plugin's BT (IsSearching/IsAccompany/
// IsConversating) and on the timeline.

class SearchState final : public RobotState {
public:
    void enter(Robot& robot) override;
    RobotStateType getType() const override { return RobotStateType::MISSION; }
    std::string getName() const override { return "search"; }
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<SearchState>(*this); }
};

class AccompanyState final : public RobotState {
public:
    void enter(Robot& robot) override;
    RobotStateType getType() const override { return RobotStateType::MISSION; }
    std::string getName() const override { return "accompany"; }
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<AccompanyState>(*this); }
};

class ConversateState final : public RobotState {
public:
    enum class Type { FOUND_PERSON, DROP_OFF };
    const Type conversationType;
    ConversateState(const Type type = Type::FOUND_PERSON) : conversationType(type) {}

    void enter(Robot& robot) override;
    RobotStateType getType() const override { return RobotStateType::MISSION; }
    std::string getName() const override { return "conversate"; }
    double getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const override;
    std::unique_ptr<RobotState> clone() const override { return std::make_unique<ConversateState>(*this); }
};

}  // namespace des
