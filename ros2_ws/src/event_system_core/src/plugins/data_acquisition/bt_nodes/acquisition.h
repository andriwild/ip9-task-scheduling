#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>

#include "model/i_sim_context.h"
#include "model/robot.h"
#include "model/event/start_drive_event.h"
#include "plugins/data_acquisition/data_acquisition_order.h"
#include "plugins/data_acquisition/events/start_acquisition_event.h"

class IsAtTargetLocation final : public BT::ConditionNode {
public:
    IsAtTargetLocation(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto& order = static_cast<DataAcquisitionOrder&>(*ctx->getOrderPtr());
        const auto robot = ctx->getRobot();
        const bool atTarget = robot->getLocation() == order.roomName && !robot->isDriving();
        RCLCPP_DEBUG(rclcpp::get_logger("BT - DataAcquisition"), "IsAtTargetLocation: %d", atTarget);
        return atTarget ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class GoToLocation final : public BT::StatefulActionNode {
public:
    GoToLocation(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus onStart() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto& order = static_cast<DataAcquisitionOrder&>(*ctx->getOrderPtr());
        RCLCPP_DEBUG(rclcpp::get_logger("BT - DataAcquisition"), "GoToLocation: -> %s", order.roomName.c_str());
        ctx->pushEvent(std::make_shared<StartDriveEvent>(ctx->getTime(), order.roomName));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto& order = static_cast<DataAcquisitionOrder&>(*ctx->getOrderPtr());
        const auto robot = ctx->getRobot();
        if (robot->getLocation() == order.roomName && !robot->isDriving()) {
            RCLCPP_DEBUG(rclcpp::get_logger("BT - DataAcquisition"), "GoToLocation: arrived at %s", order.roomName.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}
};

class ExecuteAcquisition final : public BT::StatefulActionNode {
public:
    ExecuteAcquisition(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus onStart() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        RCLCPP_INFO(rclcpp::get_logger("BT - DataAcquisition"), "ExecuteAcquisition: start");
        ctx->pushEvent(std::make_shared<StartAcquisitionEvent>(ctx->getTime()));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto order = ctx->getOrderPtr();
        if (!order || order->state == des::MissionState::COMPLETED) {
            RCLCPP_INFO(rclcpp::get_logger("BT - DataAcquisition"), "ExecuteAcquisition: done");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}
};
