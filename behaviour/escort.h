#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/basic_types.h>

#include "../model/context.h"
#include "../model/robot_state.h"
#include "../util/rnd.h"


class IsEscorting : public BT::ConditionNode {
public:
    IsEscorting(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<SimulationContext*>("ctx");
        return ctx->robot.isEscorting() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
 
class ArrivedWithPerson: public BT::ConditionNode {
public:
    ArrivedWithPerson(const std::string& name, const BT::NodeConfig& config):
    BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx        = config().blackboard.get()->get<SimulationContext*>("ctx");
        bool successful = true;
        ctx->notifyLog("Arrived with person at meeting location.");
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
        auto ctx        = config().blackboard.get()->get<SimulationContext*>("ctx");
        int currentTime = config().blackboard.get()->get<int>("current_time");
        ctx->notifyLog("Start drop off conversation!");

        ctx->queue.push(std::make_shared<DropOffConversationCompleteEvent>(currentTime + 10));
        ctx->changeRobotState(new ConversateState());
        return BT::NodeStatus::SUCCESS;
    }
};

class AbortEscort: public BT::SyncActionNode {
public:
    AbortEscort(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx"), BT::InputPort<int>("current_time") };
    }

    BT::NodeStatus tick() override {
        auto ctx        = config().blackboard.get()->get<SimulationContext*>("ctx");
        int currentTime = config().blackboard.get()->get<int>("current_time");

        ctx->notifyLog("Aborting Escort!");
        ctx->updateAppointmentState(des::MissionState::FAILED);
        ctx->changeRobotState(new IdleState());
        return BT::NodeStatus::SUCCESS;
    }
};
