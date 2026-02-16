#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>

#include "../model/context.h"
#include "../model/robot_state.h"
#include "../util/rnd.h"

class IsSearching final : public BT::ConditionNode {
public:
    IsSearching(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        if (ctx->m_robot->getStateType() == des::RobotStateType::SEARCHING) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class ScanLocation final : public BT::SyncActionNode {
public:
    ScanLocation(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        const bool personFound = rnd::uni() < ctx->getPersonFindProbability();
        if (personFound){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class StartAccompanyConversation: public BT::SyncActionNode {
public:
    StartAccompanyConversation(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        auto target = ctx->getAppointment()->roomName;
        ctx->m_queue.push(std::make_shared<StartFoundPersonConversationEvent>(ctx->getTime()));
        return BT::NodeStatus::SUCCESS;
    }
};


class HasNextLocation final : public BT::ConditionNode {
public:
    HasNextLocation(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        const auto currentState = ctx->m_robot->getState();
        const auto ss = dynamic_cast<SearchState*>(currentState);

        if (ss->locations.empty()){
            ctx->updateAppointmentState(des::MissionState::FAILED);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class MoveToNextLocation final : public BT::SyncActionNode {
public:
    MoveToNextLocation(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        const auto searchState = dynamic_cast<SearchState*>(ctx->m_robot->getState());
        const auto locations = searchState->locations;
        std::string nextLocation = locations.front();
        searchState->locations.erase(searchState->locations.begin());
        ctx->m_queue.push(std::make_shared<StartDriveEvent>(ctx->getTime(), nextLocation));
        return BT::NodeStatus::SUCCESS;
    }
};

class AbortSearch final: public BT::SyncActionNode {
public:
    AbortSearch(const std::string& name, const BT::NodeConfig& config): SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        ctx->m_queue.push(std::make_shared<AbortSearchEvent>(ctx->getTime()));
        return BT::NodeStatus::SUCCESS;
    }
};