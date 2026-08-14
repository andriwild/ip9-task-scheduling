#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include "../../../util/log.h"
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <memory>

#include <algorithm>
#include <string>
#include <vector>

#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"
#include "model/person.h"
#include "plugins/accompany/accompany_order.h"
#include "plugins/accompany/search_exclusion.h"
#include "plugins/accompany/states.h"
#include "plugins/accompany/events/start_accompany_event.h"
#include "util/constants.h"
#include "util/rnd.h"

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

// if the robot searches for a person a meets another, the robot asks for information
// only a person from the same office can get information, and it can be wrong by a given probability
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
        const Person* person = ctx->getPersonByName(order.personName);
        if (room.empty() || room == IN_TRANSIT || room == OUTDOOR || room == ctx->getRobot()->getLocation()) {
            DES_LOG_DEBUG("des.plugin.accompany.conversation", "ApplyDirections: '%s' is not a usable hint", room.c_str());
            return BT::NodeStatus::FAILURE;
        }
        // const Person* informant = ctx->getPersonByName(order.pendingAsk.front());
        // if (person->workplace != informant->workplace) {
        //     DES_LOG_DEBUG("des.plugin.accompany.conversation", "ApplyDirections: %s shares no office with %s", informant->firstName.c_str(), order.personName.c_str());
        //     return BT::NodeStatus::FAILURE;
        // }

        std::string hint = room;
        bool wrong = false;
        const double wrongProbability = ctx->getConfig()->personDirectionsWrongProbability;
        if (wrongProbability > 0.0 && rnd::uni(ctx->robotRng()) < wrongProbability) {
            std::vector<std::string> candidates;
            for (const auto& label : person->roomLabels) {
                if (label == room || label == ctx->getRobot()->getLocation() || isSearchExcluded(*ctx, label)) {
                    continue;
                }
                candidates.push_back(label);
            }
            if (candidates.empty()) {
                DES_LOG_DEBUG("des.plugin.accompany.conversation", "ApplyDirections: no wrong room available for %s", order.personName.c_str());
                return BT::NodeStatus::FAILURE;
            }
            const auto draw = static_cast<size_t>(rnd::uni(ctx->robotRng(), 0.0, static_cast<double>(candidates.size())));
            hint = candidates[std::min(draw, candidates.size() - 1)];
            wrong = true;
        }

        std::erase(order.remainingSearch, hint);
        order.remainingSearch.insert(order.remainingSearch.begin(), hint);
        if (wrong) {
            DES_LOG_DEBUG("des.plugin.accompany.conversation", "ApplyDirections: %s points wrongly to %s for %s", order.pendingAsk.front().c_str(), hint.c_str(), order.personName.c_str());
        } else {
            DES_LOG_DEBUG("des.plugin.accompany.conversation", "ApplyDirections: %s points to %s for %s", order.pendingAsk.front().c_str(), hint.c_str(), order.personName.c_str());
        }
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

