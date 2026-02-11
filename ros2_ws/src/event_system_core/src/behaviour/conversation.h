#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <qstringalgorithms.h>
#include <memory>

#include "../model/context.h"
#include "../model/robot_state.h"
#include "../model/event.h"

class IsConversating : public BT::ConditionNode {
public:
    IsConversating(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        if (ctx->m_robot->getStateType() == des::RobotStateType::CONVERSATE) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class ConversationFinished : public BT::ConditionNode {
public:
    ConversationFinished(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        auto convResult = ctx->m_robot->getState()->getResult();
        if (convResult == des::Result::SUCCESS) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class WasConversationSuccessful : public BT::ConditionNode {
public:
    WasConversationSuccessful(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        auto convResult = ctx->m_robot->getState()->getResult();
        if (convResult == des::Result::SUCCESS) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class StartAccompanyAction: public BT::SyncActionNode {
public:
    StartAccompanyAction(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        ctx->m_queue.push(std::make_shared<StartAccompanyEvent>(ctx->getTime()));
        return BT::NodeStatus::SUCCESS;
    }
};

class IsFoundPersonConversation : public BT::ConditionNode {
public:
    IsFoundPersonConversation(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        auto currentState = ctx->m_robot->getState();
        auto convState = dynamic_cast<ConversateState*>(currentState);
        
        if (convState && convState->conversationType == ConversateState::Type::FOUND_PERSON) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class IsDropOffConversation : public BT::ConditionNode {
public:
    IsDropOffConversation(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        auto currentState = ctx->m_robot->getState();
        auto convState = dynamic_cast<ConversateState*>(currentState);

        if (convState && convState->conversationType == ConversateState::Type::DROP_OFF) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class CompleteMissionAction : public BT::SyncActionNode {
public:
    CompleteMissionAction(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        auto convResult = ctx->m_robot->getState()->getResult();
        if (convResult == des::Result::SUCCESS) {
            ctx->updateAppointmentState(des::MissionState::COMPLETED);
        } else {
            ctx->updateAppointmentState(des::MissionState::FAILED);
        }
        ctx->changeRobotState(std::make_unique<IdleState>());
        ctx->m_queue.push(std::make_shared<MissionCompleteEvent>(ctx->getTime(), ctx->getAppointment()));
        
        return BT::NodeStatus::SUCCESS;
    }
};
