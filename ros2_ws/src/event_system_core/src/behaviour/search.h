#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>

#include "../model/context.h"
#include "../model/robot_state.h"
#include "../util/rnd.h"

class IsSearching : public BT::ConditionNode {
public:
    IsSearching(const std::string& name, const BT::NodeConfig& config):
        BT::ConditionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        return ctx->m_robot->isSearching() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class ScanLocation : public BT::SyncActionNode {
public:
    ScanLocation(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        bool personFound = rnd::uni() < ctx->getPersonFindProbability();
        if (personFound){
            ctx->notifyLog("Person found: " + ctx->getAppointment()->personName);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class StartAccompanyConversation: public BT::SyncActionNode {
public:
    StartAccompanyConversation(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx"), BT::InputPort<int>("current_time") };
    }

    BT::NodeStatus tick() override {
        auto ctx        = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        int currentTime = config().blackboard.get()->get<int>("current_time");

        // TODO: add time randomness
        auto target = ctx->getAppointment()->roomName;

        auto eventTime = currentTime + ctx->getRndConversationTime();
        ctx->m_queue.push(std::make_shared<FoundPersonConversationCompleteEvent>(eventTime));
        ctx->changeRobotState(std::make_unique<ConversateState>());
        return BT::NodeStatus::SUCCESS;
    }
};


class HasNextLocation: public BT::ConditionNode {
public:
    HasNextLocation(const std::string& name, const BT::NodeConfig& config):
        BT::ConditionNode(name, config) 
    {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx"), BT::InputPort<int>("current_time"), };
    }
    
    BT::NodeStatus tick() override {
        auto ctx        = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        int currentTime = config().blackboard.get()->get<int>("current_time");

        auto currentState = ctx->m_robot->getState();
        auto ss = dynamic_cast<SearchState*>(currentState);

        if (ss->locations.empty()){
            ctx->notifyLog("Person not found at any place!");
            ctx->updateAppointmentState(des::MissionState::FAILED);
            ctx->m_queue.push(std::make_shared<MissionCompleteEvent>(currentTime + 1));
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class MoveToNextLocation: public BT::SyncActionNode {
public:
    MoveToNextLocation(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx"), BT::InputPort<int>("current_time") };
    }
    
    BT::NodeStatus tick() override {
        auto ctx        = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        int currentTime = config().blackboard.get()->get<int>("current_time");

        auto searchState = dynamic_cast<SearchState*>(ctx->m_robot->getState());
        auto locations = searchState->locations;
        auto next = locations.front();
        searchState->locations.erase(searchState->locations.begin());
        
        ctx->scheduleArrival(currentTime, next);
        return BT::NodeStatus::SUCCESS;
    }
};

class AbortSearch: public BT::SyncActionNode {
public:
    AbortSearch(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx") };
    }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        ctx->notifyLog("Aborting Search!");
        ctx->updateAppointmentState(des::MissionState::FAILED);
        ctx->changeRobotState(std::make_unique<IdleState>());
        return BT::NodeStatus::SUCCESS;
    }
};
