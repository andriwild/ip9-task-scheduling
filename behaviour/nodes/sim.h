#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>

#include "behaviortree_cpp/basic_types.h"
#include "../../model/context.h"
#include "../../model/robot_state.h"
#include "../../util/rnd.h"

class IsSearching : public BT::ConditionNode {
public:
    IsSearching(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx") };
    }
    
    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<SimulationContext*>("ctx");
        return ctx->robot.isSearching() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IsEscorting : public BT::ConditionNode {
public:
    IsEscorting(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx") };
    }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<SimulationContext*>("ctx");
        return ctx->robot.isEscorting() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};


class HandleSearch : public BT::SyncActionNode {
public:
    HandleSearch(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<int>("ctx"), 
            BT::InputPort<int>("current_time"), 
            BT::InputPort<int>("location") 
        };
    }

    BT::NodeStatus tick() override {
        auto ctx             = config().blackboard.get()->get<SimulationContext*>("ctx");
        int currentTime      = config().blackboard.get()->get<int>("current_time");
        std::string location = config().blackboard.get()->get<std::string>("location");

        bool personFound = rnd::uni() > 0.5;

        if (personFound) {
            if (!ctx->getAppointment().has_value()) {
                ctx->notifyLog("[ERROR] Searching without appointment data!");
                return BT::NodeStatus::FAILURE;
            }

            ctx->notifyLog("Person found! Starting escort.");
            auto goal = ctx->getAppointment().value().roomName;
            
            ctx->changeRobotState(new EscortState());
            ctx->scheduleArrival(currentTime, goal); 
        } else {
            ctx->notifyLog("Person not in " + location + ". Checking next room...");
            auto currentState = ctx->robot.getState();

            if (auto ss = dynamic_cast<SearchState*>(currentState)){
                if(ss->locations.empty()){
                    ctx->notifyLog("Person not found at any place!");
                    ctx->changeRobotState(new MoveState());
                    ctx->scheduleArrival(currentTime, ctx->robot.getIdleLocation(), true);
                    return BT::NodeStatus::SUCCESS;
                }
                auto nextLocation = ss->locations.front(); // TODO: take nearest locations
                ss->locations.erase(ss->locations.begin());
                ctx->scheduleArrival(currentTime, nextLocation);
            } else {
                ctx->notifyLog("[ERROR] Searching without beeing in SearchState!");
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class HandleEscort : public BT::SyncActionNode {
public:
    HandleEscort(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx"), BT::InputPort<int>("current_time") };
    }

    BT::NodeStatus tick() override {
        auto ctx        = config().blackboard.get()->get<SimulationContext*>("ctx");
        int currentTime = config().blackboard.get()->get<int>("current_time");

        ctx->notifyLog("Escort finished. Returning to Dock.");
        ctx->changeRobotState(new MoveState());
        ctx->scheduleArrival(currentTime, ctx->robot.getIdleLocation(), true);
        return BT::NodeStatus::SUCCESS;
    }
};

class HandleIdle : public BT::SyncActionNode {
public:
    HandleIdle(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx"), BT::InputPort<int>("location") };
    }

    BT::NodeStatus tick() override {

        auto ctx             = config().blackboard.get()->get<SimulationContext*>("ctx");
        std::string location = config().blackboard->get<std::string>("location");

        ctx->notifyLog("Robot is idle at " + location);
        ctx->changeRobotState(new IdleState());
        return BT::NodeStatus::SUCCESS;
    }
};
