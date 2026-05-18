#pragma once

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include "../model/i_sim_context.h"

class IsInterruptActive final : public BT::ConditionNode {
public:
    IsInterruptActive(const std::string& name, const BT::NodeConfig& cfg)
        : ConditionNode(name, cfg) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard->get<std::shared_ptr<ISimContext>>("ctx");
        return ctx->hasActiveInterrupt() ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
    }
};
