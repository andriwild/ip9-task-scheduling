#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include "../util/log.h"
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <memory>

#include "engine/contracts/i_sim_context.h"
#include "engine/event.h"
#include "../model/robot.h"
#include "../plugins/order_registry.h"

namespace des {

class HasPendingOrder final : public BT::ConditionNode {
public:
    HasPendingOrder(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx        = config().blackboard.get()->get<ISimContext*>("ctx");
        const bool hasPending = ctx->hasScheduledOrder();
        DES_LOG_DEBUG("des.bt.mission_control", "HasPendingOrder: %d", hasPending);
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
        const auto ctx    = config().blackboard.get()->get<ISimContext*>("ctx");
        const bool isBusy = ctx->getRobot()->isBusy();
        DES_LOG_DEBUG("des.bt.mission_control", "IsRobotBusy: %d", isBusy);
        if (isBusy) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class IsOrderAssigned final : public BT::ConditionNode {
public:
    IsOrderAssigned(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    // SUCCESS if any order is currently assigned to the robot (state may be
    // PENDING right after accept, IN_PROGRESS during execution). Used by
    // MissionControlRoutine to avoid greedy double-accept of background
    // missions whose plugins don't change the robot state on onMissionStart.
    BT::NodeStatus tick() override {
        const auto ctx      = config().blackboard.get()->get<ISimContext*>("ctx");
        const bool assigned = ctx->getOrderPtr() != nullptr;
        DES_LOG_DEBUG("des.bt.mission_control", "IsOrderAssigned: %d", assigned);
        return assigned ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class AcceptOrderAction final : public BT::SyncActionNode {
public:
    AcceptOrderAction(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        assert(ctx->hasScheduledOrder());
        const auto order = ctx->popScheduledOrder();
        DES_LOG_DEBUG("des.bt.mission_control", "AcceptOrderAction for order %d (type=%s)",
                    order->id, order->type.c_str());
        ctx->setOrderPtr(order);
        // PENDING -> IN_PROGRESS transition happens in the plugin's StartXxxEvent,
        // so plugin Execute nodes can use state==PENDING as an idempotency check
        // for "mission physically started" on tick.
        ctx->pushEvent(std::make_shared<MissionStartEvent>(ctx->getTime(), order));
        return BT::NodeStatus::SUCCESS;
    }
};

class RejectOrderAction final : public BT::SyncActionNode {
public:
    RejectOrderAction(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");

        assert(ctx->hasScheduledOrder());
        const auto order = ctx->popScheduledOrder();
        DES_LOG_DEBUG("des.bt.mission_control",
                     "Reject mission %d (type=%s) — see preceding 'infeasible' log for the concrete deadline/slack",
                     order->id, order->type.c_str());
        order->state     = OrderState::REJECTED;
        ctx->pushEvent(std::make_shared<MissionCompleteEvent>(ctx->getTime(), order));
        return BT::NodeStatus::SUCCESS;
    }
};

class HasBackgroundOrder final : public BT::ConditionNode {
public:
    HasBackgroundOrder(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx     = config().blackboard.get()->get<ISimContext*>("ctx");
        const bool hasBg   = ctx->hasBackgroundOrder();
        DES_LOG_DEBUG("des.bt.mission_control", "HasBackgroundOrder: %d", hasBg);
        return hasBg ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class AcceptBackgroundOrderAction final : public BT::SyncActionNode {
public:
    AcceptBackgroundOrderAction(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    // Delegates selection + feasibility to the background mission pool. FAILURE = no
    // feasible mission, the outer BT falls through to charging/idle.
    BT::NodeStatus tick() override {
        const auto ctx   = config().blackboard.get()->get<ISimContext*>("ctx");
        const auto order = ctx->acceptFeasibleBackgroundOrder();
        if (!order) {
            return BT::NodeStatus::FAILURE;
        }
        ctx->publishMission(order, ctx->getTime());
        ctx->pushEvent(std::make_shared<MissionStartEvent>(ctx->getTime(), order));
        return BT::NodeStatus::SUCCESS;
    }
};

class OrderFeasibilityCheck final : public BT::SyncActionNode {
public:
    OrderFeasibilityCheck(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");

        const auto order = ctx->nextScheduledOrder();
        const auto& plugin = OrderRegistry::instance().get(order->type);
        const bool timeFeasible = plugin.isFeasible(*order, *ctx);

        const auto robot       = ctx->getRobot();
        const auto bat         = robot->batteryStats();
        const double voltage   = robot->batteryVoltage();
        const double currentWh = bat.soc * bat.capacity * voltage;
        const double reserveWh = (bat.lowThreshold / 100.0) * bat.capacity * voltage;
        const double missionWh = plugin.estimateMissionEnergy(*order, *ctx, robot->getLocation());
        const bool energyFeasible = currentWh - missionWh >= reserveWh;
        if (!energyFeasible) {
            DES_LOG_DEBUG("des.plugin",
                         "Mission %d infeasible: energy %.1fWh, mission needs %.1fWh, reserve %.1fWh — charge first",
                         order->id, currentWh, missionWh, reserveWh);
        }

        const bool isFeasible = timeFeasible && energyFeasible;
        DES_LOG_DEBUG("des.bt.mission_control", "OrderFeasibilityCheck: %d", isFeasible);
        if (isFeasible) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

}  // namespace des
