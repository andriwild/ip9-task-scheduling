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

class IsAlwaysChargeAtDock final : public BT::ConditionNode {
public:
    IsAlwaysChargeAtDock(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        return ctx->getConfig()->alwaysChargeAtDock ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IsBatteryCharged final : public BT::ConditionNode {
public:
    IsBatteryCharged(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        return ctx->getRobot()->m_bat->isFullyCharged() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IsOpportunisticallyCharging final : public BT::ConditionNode {
public:
    IsOpportunisticallyCharging(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        return ctx->getRobot()->m_opportunisticCharge ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// Available for a new mission: idle, or opportunistically charging at the dock; never while driving.
class IsAvailable final : public BT::ConditionNode {
public:
    IsAvailable(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx     = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto robot   = ctx->getRobot();
        const auto type    = robot->getStateType();
        const bool driving = robot->isDriving();
        const bool available = !driving
            && (type == des::RobotStateType::IDLE
                || (type == des::RobotStateType::CHARGING && robot->m_opportunisticCharge));
        return available ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};


class IsTaskActive final : public BT::ConditionNode {
public:
    IsTaskActive(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

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
            requestDrive(*ctx, ctx->getRobot()->getIdleLocation());
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

        if (ctx->getRobot()->getStateType() != des::RobotStateType::CHARGING) {
            ctx->changeRobotState(std::make_unique<ChargeState>());
        }

        if (ctx->getRobot()->m_batteryFullEventScheduled) {
            return BT::NodeStatus::SUCCESS;
        }

        ctx->getRobot()->m_opportunisticCharge = !ctx->getRobot()->m_bat->isBatteryLow();

        const double netChargingPower = ctx->getConfig()->chargingRate - ctx->getConfig()->energyConsumptionBase;
        const double timeToFull = ctx->getRobot()->m_bat->timeToFull(netChargingPower);

        ctx->pushEvent(std::make_shared<BatteryFullEvent>(static_cast<int>(ctx->getTime() + timeToFull)));

        const double timeToTransition = ctx->getRobot()->m_bat->timeToPhaseTransition(netChargingPower);
        if (timeToTransition >= 0.0) {
            ctx->pushEvent(std::make_shared<ChargePhaseTransitionEvent>(static_cast<int>(ctx->getTime() + timeToTransition)));
            DES_LOG_INFO(rclcpp::get_logger("des.bt.charge"), "Phase transition in: %.1fs", timeToTransition);
        }

        ctx->getRobot()->m_batteryFullEventScheduled = true;
        ctx->notifyChargeStarted();
        DES_LOG_INFO(rclcpp::get_logger("des.bt.charge"), "Start Charging, time to full: %.1fs (opportunistic: %d)", timeToFull, ctx->getRobot()->m_opportunisticCharge);
        return BT::NodeStatus::SUCCESS;
    }
};
