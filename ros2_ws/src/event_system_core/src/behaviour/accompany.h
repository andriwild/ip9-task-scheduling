#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>

#include "../model/context.h"
#include "../model/robot_state.h"


class IsAccompany : public BT::ConditionNode {
public:
    IsAccompany(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        return ctx->m_robot->isAccompany() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
 
class ArrivedWithPerson: public BT::ConditionNode {
public:
    ArrivedWithPerson(const std::string& name, const BT::NodeConfig& config):
    BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        bool successful = true;
        return successful ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class StartDropoffConversation: public BT::SyncActionNode {
public:
    StartDropoffConversation(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx"), BT::InputPort<int>("current_time") };
    }

    BT::NodeStatus tick() override {
        auto ctx        = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        int currentTime = config().blackboard.get()->get<int>("current_time");

        ctx->m_queue.push(std::make_shared<StartDropOffConversationeEvent>(currentTime));
        ctx->changeRobotState(std::make_unique<ConversateState>());
        return BT::NodeStatus::SUCCESS;
    }
};

class AbortAccompany: public BT::SyncActionNode {
public:
    AbortAccompany(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx") };
    }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        ctx->updateAppointmentState(des::MissionState::FAILED);
        ctx->changeRobotState(std::make_unique<IdleState>());
        return BT::NodeStatus::SUCCESS;
    }
};
