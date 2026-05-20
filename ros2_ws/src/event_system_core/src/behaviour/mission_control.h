#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <memory>

#include "../model/i_sim_context.h"
#include "../model/event.h"
#include "../plugins/order_registry.h"

class HasPendingMission final : public BT::ConditionNode {
public:
    HasPendingMission(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx        = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool hasPending = ctx->hasPendingMission();
        RCLCPP_DEBUG(rclcpp::get_logger("BT - MissionControlRoutine"), "HasPendingMission: %d", hasPending);
        if (hasPending) { return BT::NodeStatus::SUCCESS; }
        return BT::NodeStatus::FAILURE;
    }
};

class IsRobotBusy final : public BT::ConditionNode {
public:
    IsRobotBusy(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx    = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool isBusy = ctx->getRobot()->isBusy();
        RCLCPP_DEBUG(rclcpp::get_logger("BT - MissionControlRoutine"), "IsRobotBusy: %d", isBusy);
        if (isBusy) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class IsMissionAssigned final : public BT::ConditionNode {
public:
    IsMissionAssigned(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    // SUCCESS if any order is currently assigned to the robot (state may be
    // PENDING right after accept, IN_PROGRESS during execution). Used by
    // MissionControlRoutine to avoid greedy double-accept of background
    // missions whose plugins don't change the robot state on onMissionStart.
    BT::NodeStatus tick() override {
        const auto ctx      = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool assigned = ctx->getOrderPtr() != nullptr;
        RCLCPP_DEBUG(rclcpp::get_logger("BT - MissionControlRoutine"), "IsMissionAssigned: %d", assigned);
        return assigned ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class AcceptMissionAction final : public BT::SyncActionNode {
public:
    AcceptMissionAction(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        assert(ctx->hasPendingMission());
        const auto order = ctx->popPendingMission();
        RCLCPP_INFO(rclcpp::get_logger("BT - MissionControlRoutine"), "AcceptMissionAction for order %d (type=%s)",
                    order->id, order->type.c_str());
        ctx->setOrderPtr(order);
        // PENDING -> IN_PROGRESS transition happens in the plugin's StartXxxEvent,
        // so plugin Execute nodes can use state==PENDING as an idempotency check
        // for "mission physically started" on tick.
        ctx->pushEvent(std::make_shared<MissionStartEvent>(ctx->getTime(), order));
        return BT::NodeStatus::SUCCESS;
    }
};

class RejectMissionAction final : public BT::SyncActionNode {
public:
    RejectMissionAction(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        RCLCPP_WARN(rclcpp::get_logger("BT - MissionControlRoutine"), "RejectMissionAction");

        assert(ctx->hasPendingMission());
        const auto order = ctx->popPendingMission();
        order->state     = des::MissionState::REJECTED;
        ctx->pushEvent(std::make_shared<MissionCompleteEvent>(ctx->getTime(), order));
        return BT::NodeStatus::SUCCESS;
    }
};

class HasBackgroundMission final : public BT::ConditionNode {
public:
    HasBackgroundMission(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx     = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool hasBg   = ctx->hasBackgroundMission();
        RCLCPP_DEBUG(rclcpp::get_logger("BT - MissionControlRoutine"), "HasBackgroundMission: %d", hasBg);
        return hasBg ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class AcceptBackgroundMissionAction final : public BT::SyncActionNode {
public:
    AcceptBackgroundMissionAction(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        assert(ctx->hasBackgroundMission());
        const auto order = ctx->popBackgroundMission();
        RCLCPP_INFO(rclcpp::get_logger("BT - MissionControlRoutine"),
                    "AcceptBackgroundMissionAction for order %d (type=%s)",
                    order->id, order->type.c_str());
        ctx->setOrderPtr(order);
        ctx->pushEvent(std::make_shared<MissionStartEvent>(ctx->getTime(), order));
        return BT::NodeStatus::SUCCESS;
    }
};

class MissionFeasibilityCheck final : public BT::SyncActionNode {
public:
    MissionFeasibilityCheck(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        const auto order = ctx->nextPendingMission();
        const auto& plugin = OrderRegistry::instance().get(order->type);
        const bool isFeasible  = plugin.isFeasible(*order, *ctx);
        RCLCPP_DEBUG(rclcpp::get_logger("BT - MissionControlRoutine"), "MissionFeasibilityCheck: %d", isFeasible);
        if (isFeasible) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};
