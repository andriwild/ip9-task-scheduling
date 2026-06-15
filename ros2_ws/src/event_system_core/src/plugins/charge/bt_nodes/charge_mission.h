#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>

#include "../../../util/log.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "model/event/start_drive_event.h"
#include "plugins/charge/events/start_charge_event.h"

class ChargeMissionIsAtDock final : public BT::ConditionNode {
public:
    ChargeMissionIsAtDock(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx   = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto robot = ctx->getRobot();
        const bool atDock = robot->getLocation() == robot->getIdleLocation() && !robot->isDriving();
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.charge"), "ChargeMissionIsAtDock: %d", atDock);
        return atDock ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class ChargeMissionGoToDock final : public BT::StatefulActionNode {
public:
    ChargeMissionGoToDock(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus onStart() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        if (ctx->getRobot()->isDriving()) {
            return BT::NodeStatus::RUNNING;  // resumed mid-drive after interrupt
        }
        const auto dock = ctx->getRobot()->getIdleLocation();
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.charge"), "ChargeMissionGoToDock: -> %s", dock.c_str());
        ctx->pushEvent(std::make_shared<StartDriveEvent>(ctx->getTime(), dock));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto ctx   = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto robot = ctx->getRobot();
        if (robot->getLocation() == robot->getIdleLocation() && !robot->isDriving()) {
            DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.charge"), "ChargeMissionGoToDock: arrived at dock");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}
};

class ExecuteChargeMission final : public BT::StatefulActionNode {
public:
    ExecuteChargeMission(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus onStart() override {
        const auto ctx   = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto order = ctx->getOrderPtr();
        // Idempotent on resume after interrupt: only start once.
        if (order && order->state == des::MissionState::PENDING) {
            DES_LOG_INFO(rclcpp::get_logger("des.plugin.charge"), "ExecuteChargeMission: start");
            ctx->pushEvent(std::make_shared<StartChargeEvent>(ctx->getTime(), order));
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto ctx   = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto order = ctx->getOrderPtr();
        if (!order || order->state == des::MissionState::COMPLETED) {
            DES_LOG_INFO(rclcpp::get_logger("des.plugin.charge"), "ExecuteChargeMission: done");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}
};
