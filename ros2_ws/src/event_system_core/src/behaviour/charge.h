#pragma once

#include <behaviortree_cpp/basic_types.h>
#include "../util/log.h"
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <memory>

#include "../model/i_sim_context.h"
#include "../model/robot.h"
#include "../model/event.h"

class IsBatteryLow final : public BT::ConditionNode {
public:
    IsBatteryLow(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool chargingRequired = ctx->getRobot()->updateAndGetChargingRequired();
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.charge"), "IsBatteryLow: %d", chargingRequired);
        return chargingRequired ? BT::NodeStatus::SUCCESS: BT::NodeStatus::FAILURE;
    }
};

class IsCharging final : public BT::ConditionNode {
public:
    IsCharging(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx        = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool isCharging = ctx->getRobot()->getState()->getType() == des::RobotStateType::CHARGING;
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.charge.is_charging"), "%d", isCharging);
        return isCharging ? BT::NodeStatus::SUCCESS: BT::NodeStatus::FAILURE;
    }
};


class IsTaskActive final : public BT::ConditionNode {
public:
    IsTaskActive(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    // SUCCESS while any order is assigned to the robot (PENDING or IN_PROGRESS).
    // Single source of truth = the context's current order, decoupled from
    // robot-state. Plugins no longer need to set a "task-active" RobotState to
    // get the ChargeRoutine short-circuit behavior.
    BT::NodeStatus tick() override {
        const auto ctx          = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool isTaskActive = ctx->getOrderPtr() != nullptr;
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.charge"), "IsTaskActive: %d", isTaskActive);
        return isTaskActive? BT::NodeStatus::SUCCESS: BT::NodeStatus::FAILURE;
    }
};


class GoToDock final : public BT::SyncActionNode {
public:
    GoToDock(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        DES_LOG_DEBUG(rclcpp::get_logger("des.bt.charge"), "GoToDock");
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        ctx->changeRobotState(std::make_unique<ChargeState>());
        if (ctx->getRobot()->getLocation() == ctx->getRobot()->getIdleLocation()) {
            DES_LOG_DEBUG(rclcpp::get_logger("des.bt.charge"), "Docking check: already at dock");
            return BT::NodeStatus::SUCCESS;
        }
        if (!ctx->getRobot()->isDriving()) {
            ctx->pushEvent(std::make_shared<StartDriveEvent>(ctx->getTime(), ctx->getRobot()->getIdleLocation()));
            DES_LOG_DEBUG(rclcpp::get_logger("des.bt.charge"), "Not at dock, start driving to dock");
        }
        return BT::NodeStatus::FAILURE;
    }
};

class Charge final : public BT::SyncActionNode {
public:
    Charge(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        assert(ctx->getRobot()->getLocation() == ctx->getRobot()->getIdleLocation());

        if (ctx->getRobot()->m_batteryFullEventScheduled) {
            return BT::NodeStatus::SUCCESS;
        }

        const double netChargingPower = ctx->getConfig()->chargingRate - ctx->getConfig()->energyConsumptionBase;
        const double timeToFull = ctx->getRobot()->m_bat->timeToFull(netChargingPower);

        ctx->pushEvent(std::make_shared<BatteryFullEvent>(static_cast<int>(ctx->getTime() + timeToFull)));
        ctx->getRobot()->m_batteryFullEventScheduled = true;
        DES_LOG_INFO(rclcpp::get_logger("des.bt.charge"), "Start Charging, time to full: %.1fs", timeToFull);
        return BT::NodeStatus::SUCCESS;
    }
};
