#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>

#include "../model/context.h"
#include "../model/robot_state.h"


class IsAccompany final : public BT::ConditionNode {
public:
    IsAccompany(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        const bool isArrived = ctx->m_robot->getLocation() == ctx->m_robot->getTargetLocation();
        if (ctx->m_robot->getStateType() == des::RobotStateType::ACCOMPANY && isArrived) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};
 
class ArrivedWithPerson final : public BT::ConditionNode {
public:
    ArrivedWithPerson(const std::string& name, const BT::NodeConfig& config): ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        return ctx->m_robot->getState()->getResult() == des::Result::SUCCESS
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }
};

class StartDropOffConversation final : public BT::SyncActionNode {
public:
    StartDropOffConversation(const std::string& name, const BT::NodeConfig& config): SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx"), BT::InputPort<int>("current_time") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        ctx->m_queue.push(std::make_shared<StartDropOffConversationeEvent>(ctx->getTime()));
        ctx->changeRobotState(std::make_unique<ConversateState>());
        return BT::NodeStatus::SUCCESS;
    }
};

class AbortAccompany final : public BT::SyncActionNode {
public:
    AbortAccompany(const std::string& name, const BT::NodeConfig& config): SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        ctx->updateAppointmentState(des::MissionState::FAILED);
        ctx->changeRobotState(std::make_unique<IdleState>());
        return BT::NodeStatus::SUCCESS;
    }
};
