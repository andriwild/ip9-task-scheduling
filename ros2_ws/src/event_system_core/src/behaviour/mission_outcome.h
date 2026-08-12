#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <memory>

#include "../model/robot_state.h"
#include "../util/log.h"
#include "engine/contracts/i_sim_context.h"
#include "engine/event/mission_complete_event.h"

namespace des {

inline void finishMission(ISimContext& ctx, const OrderState state) {
    DES_LOG_DEBUG("des.bt.mission_outcome", "Finish mission as %s", orderStateStr(state).c_str());
    ctx.updateOrderState(state);
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.pushEvent(std::make_shared<MissionCompleteEvent>(ctx.getTime(), ctx.getOrderPtr()));
}

class CompleteMission final : public BT::SyncActionNode {
public:
    CompleteMission(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        finishMission(*ctx, OrderState::COMPLETED);
        return BT::NodeStatus::SUCCESS;
    }
};

class FailMission final : public BT::SyncActionNode {
public:
    FailMission(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        finishMission(*ctx, OrderState::FAILED);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace des
