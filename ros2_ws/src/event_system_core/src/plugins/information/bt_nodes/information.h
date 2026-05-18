#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "model/i_sim_context.h"
#include "plugins/information/events/start_information_event.h"

class ExecuteInformation final : public BT::StatefulActionNode {
public:
    ExecuteInformation(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    // onStart and onRunning share the same logic: each tick must check whether m_current
    // is a NEW interrupt order (state==PENDING). BTCPP4 doesn't re-call onStart when
    // m_current changes under the node (e.g. when a new interrupt is stacked on top).
    BT::NodeStatus onStart() override   { return tickInternal(); }
    BT::NodeStatus onRunning() override { return tickInternal(); }
    void onHalted() override {}

private:
    BT::NodeStatus tickInternal() {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto order = ctx->getOrderPtr();
        if (!order) return BT::NodeStatus::FAILURE;

        if (order->state == des::MissionState::PENDING) {
            RCLCPP_INFO(rclcpp::get_logger("BT - Information"), "ExecuteInformation: start (order id=%d)", order->id);
            ctx->pushEvent(std::make_shared<StartInformationEvent>(ctx->getTime(), order));
            return BT::NodeStatus::RUNNING;
        }
        if (order->state == des::MissionState::COMPLETED) {
            RCLCPP_INFO(rclcpp::get_logger("BT - Information"), "ExecuteInformation: done (order id=%d)", order->id);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
};
