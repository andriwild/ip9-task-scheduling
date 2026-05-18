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

    BT::NodeStatus onStart() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        RCLCPP_INFO(rclcpp::get_logger("BT - Information"), "ExecuteInformation: start");
        ctx->pushEvent(std::make_shared<StartInformationEvent>(ctx->getTime()));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto order = ctx->getOrderPtr();
        if (!order || order->state == des::MissionState::COMPLETED) {
            RCLCPP_INFO(rclcpp::get_logger("BT - Information"), "ExecuteInformation: done");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}
};
