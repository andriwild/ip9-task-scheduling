#pragma once

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include "../model/i_sim_context.h"
#include "../model/robot_state.h"

class IsActiveOrderType final : public BT::ConditionNode {
public:
    IsActiveOrderType(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("type"), BT::InputPort<int>("ctx") }; 
    }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        auto inputType = getInput<std::string>("type");

        const auto order = ctx->getOrderPtr();
        if (!order) return BT::NodeStatus::FAILURE;

        RCLCPP_DEBUG(rclcpp::get_logger("BT - IsActiveOrderType"), "Check order type: %s", inputType->c_str());
        if (inputType.value() == order->type) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};
