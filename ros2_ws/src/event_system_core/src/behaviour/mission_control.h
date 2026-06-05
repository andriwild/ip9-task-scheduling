#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include "../util/log.h"
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <memory>

#include "../model/i_sim_context.h"
#include "../model/event.h"
#include "../model/robot.h"
#include "../plugins/order_registry.h"

class HasPendingMission final : public BT::ConditionNode {
public:
    HasPendingMission(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx        = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool hasPending = ctx->hasScheduledMission();
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.mission_control"), "HasPendingMission: %d", hasPending);
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
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.mission_control"), "IsRobotBusy: %d", isBusy);
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
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.mission_control"), "IsMissionAssigned: %d", assigned);
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
        assert(ctx->hasScheduledMission());
        const auto order = ctx->popScheduledMission();
        DES_LOG_INFO(rclcpp::get_logger("des.bt.mission_control"), "AcceptMissionAction for order %d (type=%s)",
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

        assert(ctx->hasScheduledMission());
        const auto order = ctx->popScheduledMission();
        DES_LOG_WARN(rclcpp::get_logger("des.bt.mission_control"),
                     "Reject mission %d (type=%s) — see preceding 'infeasible' log for the concrete deadline/slack",
                     order->id, order->type.c_str());
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
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.mission_control"), "HasBackgroundMission: %d", hasBg);
        return hasBg ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class AcceptBackgroundMissionAction final : public BT::SyncActionNode {
public:
    AcceptBackgroundMissionAction(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    // Delegates selection + feasibility to the background mission pool. FAILURE = no
    // feasible mission, the outer BT falls through to charging/idle.
    BT::NodeStatus tick() override {
        const auto ctx   = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto order = ctx->acceptFeasibleBackgroundMission();
        if (!order) {
            return BT::NodeStatus::FAILURE;
        }
        ctx->publishMission(order, ctx->getTime());
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

        const auto order = ctx->nextScheduledMission();
        const auto& plugin = OrderRegistry::instance().get(order->type);
        const bool isFeasible  = plugin.isFeasible(*order, *ctx);
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.mission_control"), "MissionFeasibilityCheck: %d", isFeasible);
        if (isFeasible) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};
