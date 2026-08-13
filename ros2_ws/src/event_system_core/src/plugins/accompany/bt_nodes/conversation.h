#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include "../../../util/log.h"
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <memory>

#include <algorithm>
#include <string>

#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"
#include "plugins/accompany/accompany_order.h"
#include "plugins/accompany/states.h"
#include "plugins/accompany/events/start_accompany_event.h"
#include "util/constants.h"

namespace des {

class IsConversating final : public BT::ConditionNode {
public:
    IsConversating(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const bool isConversating = dynamic_cast<ConversationState*>(ctx->getRobot()->getState()) != nullptr;
        DES_LOG_DEBUG("des.plugin.accompany.conversation", "IsConversating: %d", isConversating);
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
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");

        const auto convResult = ctx->getRobot()->getState()->getResult();
        DES_LOG_DEBUG("des.plugin.accompany.conversation", "ConversationFinished (result: %d)", static_cast<int>(convResult));
        if (convResult == Result::RUNNING) {
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
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");

        const auto convResult = ctx->getRobot()->getState()->getResult();
        DES_LOG_DEBUG("des.plugin.accompany.conversation", "WasConversationSuccessful: %s", convResult == Result::SUCCESS ? "Yes" : "No");
        if (convResult == Result::SUCCESS) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class ApplyDirections final : public BT::SyncActionNode {
public:
    ApplyDirections(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        auto& order = static_cast<AccompanyOrder&>(*ctx->getOrderPtr());
        if (order.pendingAsk.empty()) {
            return BT::NodeStatus::FAILURE;
        }

        const std::string room = ctx->getPersonLocation(order.personName);
        if (room.empty() || room == IN_TRANSIT || room == OUTDOOR || room == ctx->getRobot()->getLocation()) {
            DES_LOG_DEBUG("des.plugin.accompany.conversation", "ApplyDirections: '%s' is not a usable hint", room.c_str());
            return BT::NodeStatus::FAILURE;
        }
        std::erase(order.remainingSearch, room);
        order.remainingSearch.insert(order.remainingSearch.begin(), room);
        DES_LOG_DEBUG("des.plugin.accompany.conversation", "ApplyDirections: %s points to %s for %s", order.pendingAsk.front().c_str(), room.c_str(), order.personName.c_str());
        return BT::NodeStatus::SUCCESS;
    }
};

class ResumeSearchAfterAsk final : public BT::SyncActionNode {
public:
    ResumeSearchAfterAsk(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        auto& order = static_cast<AccompanyOrder&>(*ctx->getOrderPtr());

        if (!order.pendingAsk.empty()) {
            order.pendingAsk.pop_front();
        }
        order.phase = AccompanyPhase::SEARCH;
        ctx->changeRobotState(std::make_unique<SearchState>());
        DES_LOG_DEBUG("des.plugin.accompany.conversation", "ResumeSearchAfterAsk: %zu still waiting", order.pendingAsk.size());
        return BT::NodeStatus::SUCCESS;
    }
};

class StartAccompanyAction final : public BT::SyncActionNode {
public:
    StartAccompanyAction(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        ctx->pushEvent(std::make_shared<StartAccompanyEvent>(ctx->getTime(), ctx->getOrderPtr()));
        DES_LOG_DEBUG("des.plugin.accompany.conversation", "Start Accompany Action");
        return BT::NodeStatus::SUCCESS;
    }
};

class IsConversationKind final : public BT::ConditionNode {
public:
    IsConversationKind(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("kind"), BT::InputPort<int>("ctx") };
    }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const auto inputKind = getInput<std::string>("kind");
        if (!inputKind.has_value()) {
            return BT::NodeStatus::FAILURE;
        }
        const auto expected = conversationKindFromString(inputKind.value());
        const auto convState = dynamic_cast<ConversationState*>(ctx->getRobot()->getState());

        const bool matches = expected.has_value() && convState != nullptr && convState->kind == expected.value();
        DES_LOG_DEBUG("des.plugin.accompany.conversation", "IsConversationKind %s: %d", inputKind->c_str(), matches);
        if (matches) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

}  // namespace des

