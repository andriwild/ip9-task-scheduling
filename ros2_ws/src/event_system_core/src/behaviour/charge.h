#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <memory>

#include "../model/context.h"

class IsBatteryLow final : public BT::ConditionNode {
public:
    IsBatteryLow(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        if (ctx->m_robot->updateAndGetChargingRequired()) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class Charge final : public BT::StatefulActionNode {
public:
    Charge(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus onStart() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        
        if (!ctx->m_robot->updateAndGetChargingRequired()) {
            return BT::NodeStatus::SUCCESS;
        }

        // If not charging yet, start charging and schedule event
        if (ctx->m_robot->getStateType() != des::RobotStateType::CHARGING) {
             const double netChargingPower = ctx->getConfig()->chargingRate - ctx->getConfig()->energyConsumptionBase;
             const double timeToFull = ctx->m_robot->m_bat->timeToFull(netChargingPower);
             
             if (timeToFull > 0) {
                 ctx->m_queue.push(std::make_shared<BatteryFullEvent>(static_cast<int>(ctx->getTime() + timeToFull)));
                 ctx->changeRobotState(std::make_unique<ChargeState>());
             } else {
                 ctx->m_robot->setChargingRequired(false);
                 return BT::NodeStatus::SUCCESS;
             }
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        if (!ctx->m_robot->updateAndGetChargingRequired()) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        // Optional: Handle interruption (e.g. stop charging if needed, though state change handles it generally)
    }
};
