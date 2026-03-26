#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <memory>

#include "../model/i_sim_context.h"

class IsBatteryLow final : public BT::ConditionNode {
public:
    IsBatteryLow(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool chargingRequired = ctx->getRobot()->updateAndGetChargingRequired();
        RCLCPP_DEBUG(rclcpp::get_logger("BT - ChargeRoutine"), "IsBatteryLow: %d", chargingRequired);
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
        RCLCPP_DEBUG(rclcpp::get_logger("BT - IsCharging"), "%d", isCharging);
        return isCharging ? BT::NodeStatus::SUCCESS: BT::NodeStatus::FAILURE;
    }
};


class IsTaskActive final : public BT::ConditionNode {
public:
    IsTaskActive(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx          = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool isTaskActive = ctx->getRobot()->isTaskActive();
        RCLCPP_DEBUG(rclcpp::get_logger("BT - IsCharging"), "%d", isTaskActive);
        return isTaskActive? BT::NodeStatus::SUCCESS: BT::NodeStatus::FAILURE;
    }
};


class GoToDock final : public BT::SyncActionNode {
public:
    GoToDock(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        std::cout << "GoToDock" << std::endl;
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        ctx->changeRobotState(std::make_unique<ChargeState>());
        if (ctx->getRobot()->getLocation() == ctx->getRobot()->getIdleLocation()) {
            RCLCPP_DEBUG(rclcpp::get_logger("BT - ChargeRoutine"), "Docking check: already at dock");
            return BT::NodeStatus::SUCCESS;
        }
        if (!ctx->getRobot()->isDriving()) {
            ctx->pushEvent(std::make_shared<StartDriveEvent>(ctx->getTime(), ctx->getRobot()->getIdleLocation()));
            RCLCPP_DEBUG(rclcpp::get_logger("BT - ChargeRoutine"), "Not at dock, start driving to dock");
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
        RCLCPP_INFO(rclcpp::get_logger("BT - ChargeRoutine"), "Start Charging, time to full: %.1fs", timeToFull);
        return BT::NodeStatus::SUCCESS;
    }
};
