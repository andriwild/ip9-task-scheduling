#pragma once

#include <behaviortree_cpp/basic_types.h>
#include "../util/log.h"
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
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.idle"), "IsIdle: %d", isIdle);
        if (isIdle) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};


class IsReturning final : public BT::ConditionNode {
public:
    IsReturning(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard->get<std::shared_ptr<ISimContext>>("ctx");
        return ctx->getRobot()->getStateType() == des::RobotStateType::RETURNING
            ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class Docking final : public BT::SyncActionNode {
public:
    Docking(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        if (ctx->getRobot()->getLocation() == ctx->getRobot()->getIdleLocation()) {
            DES_LOG_INFO(rclcpp::get_logger("des.bt.idle"), "Docking: already at dock");
            return BT::NodeStatus::SUCCESS;
        }
        // If the robot is already driving back, do nothing — the in-flight StopDriveEvent
        // will trigger the next tick.
        if (!ctx->getRobot()->isDriving()) {
            ctx->changeRobotState(std::make_unique<ReturningState>());
            ctx->pushEvent(std::make_shared<StartDriveEvent>(ctx->getTime(), ctx->getRobot()->getIdleLocation()));
            DES_LOG_INFO(rclcpp::get_logger("des.bt.idle"), "Not at dock — start driving to %s", ctx->getRobot()->getIdleLocation().c_str());
        } else {
            DES_LOG_INFO(rclcpp::get_logger("des.bt.idle"), "Already driving back to dock");
        }
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
        DES_LOG_INFO(rclcpp::get_logger("des.bt.idle"), "Enter Idle");
        return BT::NodeStatus::SUCCESS;
    }
};

class HasPendingMissionIdle final : public BT::ConditionNode {
public:
    HasPendingMissionIdle(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool hasPending = ctx->hasScheduledMission();
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.idle"), "HasPendingMissionIdle: %d", hasPending);
        if (hasPending) { return BT::NodeStatus::SUCCESS; }
        return BT::NodeStatus::FAILURE;
    }
};
