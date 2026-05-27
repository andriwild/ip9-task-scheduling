#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include "../../../util/log.h"
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>

#include "model/i_sim_context.h"
#include "model/robot.h"
#include "model/event/start_drive_event.h"
#include "plugins/clean/clean_order.h"
#include "plugins/clean/events/start_clean_event.h"

class CleanIsAtTargetLocation final : public BT::ConditionNode {
public:
    CleanIsAtTargetLocation(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto& order = dynamic_cast<CleanOrder&>(*ctx->getOrderPtr());
        const auto robot = ctx->getRobot();
        const bool atTarget = robot->getLocation() == order.roomName && !robot->isDriving();
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.clean"), "CleanIsAtTargetLocation: %d", atTarget);
        return atTarget ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class CleanGoToLocation final : public BT::StatefulActionNode {
public:
    CleanGoToLocation(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus onStart() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        if (ctx->getRobot()->isDriving()) {
            return BT::NodeStatus::RUNNING;  // resumed mid-drive after interrupt
        }
        const auto& order = static_cast<CleanOrder&>(*ctx->getOrderPtr());
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.clean"), "CleanGoToLocation: -> %s", order.roomName.c_str());
        ctx->pushEvent(std::make_shared<StartDriveEvent>(ctx->getTime(), order.roomName));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto& order = static_cast<CleanOrder&>(*ctx->getOrderPtr());
        const auto robot = ctx->getRobot();
        if (robot->getLocation() == order.roomName && !robot->isDriving()) {
            DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.clean"), "CleanGoToLocation: arrived at %s", order.roomName.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}
};

class ExecuteClean final : public BT::StatefulActionNode {
public:
    ExecuteClean(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus onStart() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto order = ctx->getOrderPtr();
        // Idempotent on resume after interrupt: only push StartCleanEvent if mission hasn't been started yet.
        if (order && order->state == des::MissionState::PENDING) {
            DES_LOG_INFO(rclcpp::get_logger("des.plugin.clean"), "ExecuteClean: start");
            ctx->pushEvent(std::make_shared<StartCleanEvent>(ctx->getTime(), order));
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto order = ctx->getOrderPtr();
        if (!order || order->state == des::MissionState::COMPLETED) {
            DES_LOG_INFO(rclcpp::get_logger("des.plugin.clean"), "ExecuteClean: done");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}
};
