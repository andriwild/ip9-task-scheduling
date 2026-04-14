#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <memory>

#include "../model/i_sim_context.h"
#include "../model/robot_state.h"
#include "../model/robot.h"
#include "../model/event.h"

class IsIdle final : public BT::ConditionNode {
public:
    IsIdle(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        const bool isIdle = ctx->getRobot()->getStateType() == des::RobotStateType::IDLE;
        RCLCPP_DEBUG(rclcpp::get_logger("BT - IdleRoutine"), "IsIdle: %d", isIdle);
        if (isIdle) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};


class Docking final : public BT::SyncActionNode {
public:
    Docking(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        if (ctx->getRobot()->getLocation() == ctx->getRobot()->getIdleLocation()) {
            RCLCPP_DEBUG(rclcpp::get_logger("BT - IdleRoutine"), "Docking check: already at dock");
            return BT::NodeStatus::SUCCESS;
        }
        ctx->changeRobotState(std::make_unique<IdleState>());
        ctx->pushEvent(std::make_shared<StartDriveEvent>(ctx->getTime(), ctx->getRobot()->getIdleLocation()));
        RCLCPP_INFO(rclcpp::get_logger("BT - IdleRoutine"), "Not at dock, start driving to dock");
        return BT::NodeStatus::FAILURE;
    }
};

class EnterIdle final : public BT::ConditionNode {
public:
    EnterIdle(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) { }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        assert(ctx->getRobot()->getLocation() == ctx->getRobot()->getIdleLocation());
        ctx->changeRobotState(std::make_unique<IdleState>());
        RCLCPP_INFO(rclcpp::get_logger("BT - IdleRoutine"), "Enter Idle");
        return BT::NodeStatus::SUCCESS;
    }
};

class HasPendingMissionIdle final : public BT::ConditionNode {
public:
    HasPendingMissionIdle(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool hasPending = ctx->hasPendingMission();
        RCLCPP_DEBUG(rclcpp::get_logger("BT - IdleRoutine"), "HasPendingMissionIdle: %d", hasPending);
        if (hasPending) { return BT::NodeStatus::SUCCESS; }
        return BT::NodeStatus::FAILURE;
    }
};
