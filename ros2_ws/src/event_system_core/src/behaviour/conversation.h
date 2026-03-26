#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <memory>

#include "../model/i_sim_context.h"
#include "../model/robot_state.h"
#include "../model/event.h"

class IsConversating final : public BT::ConditionNode {
public:
    IsConversating(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const bool isConversating = ctx->getRobot()->getStateType() == des::RobotStateType::CONVERSATE;
        RCLCPP_DEBUG(rclcpp::get_logger("BT - ConversateRoutine"), "IsConversating: %d", isConversating);
        if (isConversating) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class ConversationFinished final : public BT::ConditionNode {
public:
    ConversationFinished(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        const auto convResult = ctx->getRobot()->getState()->getResult();
        RCLCPP_DEBUG(rclcpp::get_logger("BT - ConversateRoutine"), "ConversationFinished (result: %d)", static_cast<int>(convResult));
        if (convResult == des::Result::RUNNING) {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class WasConversationSuccessful final : public BT::ConditionNode {
public:
    WasConversationSuccessful(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        const auto convResult = ctx->getRobot()->getState()->getResult();
        RCLCPP_INFO(rclcpp::get_logger("BT - ConversateRoutine"), "WasConversationSuccessful: %s", convResult == des::Result::SUCCESS ? "Yes" : "No");
        if (convResult == des::Result::SUCCESS) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class StartAccompanyAction final : public BT::SyncActionNode {
public:
    StartAccompanyAction(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        ctx->pushEvent(std::make_shared<StartAccompanyEvent>(ctx->getTime()));
        RCLCPP_INFO(rclcpp::get_logger("BT - ConversateRoutine"), "Start Accompany Action");
        return BT::NodeStatus::SUCCESS;
    }
};

class IsFoundPersonConversation final : public BT::ConditionNode {
public:
    IsFoundPersonConversation(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto currentState = ctx->getRobot()->getState();
        const auto convState = dynamic_cast<ConversateState*>(currentState);
        
        const bool isFoundPerson = convState && convState->conversationType == ConversateState::Type::FOUND_PERSON;
        RCLCPP_DEBUG(rclcpp::get_logger("BT - ConversateRoutine"), "IsFoundPersonConversation: %d", isFoundPerson);
        if (isFoundPerson) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class IsDropOffConversation final : public BT::ConditionNode {
public:
    IsDropOffConversation(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        const auto currentState = ctx->getRobot()->getState();
        const auto convState = dynamic_cast<ConversateState*>(currentState);

        const bool isDropOff = convState && convState->conversationType == ConversateState::Type::DROP_OFF;
        RCLCPP_DEBUG(rclcpp::get_logger("BT - ConversateRoutine"), "IsDropOffConversation: %d", isDropOff);
        if (isDropOff) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class CompleteMissionAction final : public BT::SyncActionNode {
public:
    CompleteMissionAction(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        const auto convResult = ctx->getRobot()->getState()->getResult();
        if (convResult == des::Result::SUCCESS) {
            ctx->updateAppointmentState(des::MissionState::COMPLETED);
        } else {
            ctx->updateAppointmentState(des::MissionState::FAILED);
        }
        ctx->changeRobotState(std::make_unique<IdleState>());
        ctx->pushEvent(std::make_shared<MissionCompleteEvent>(ctx->getTime(), ctx->getAppointment()));
        RCLCPP_INFO(rclcpp::get_logger("BT - ConversateRoutine"), "Complete Mission Action");
        
        return BT::NodeStatus::SUCCESS;
    }
};