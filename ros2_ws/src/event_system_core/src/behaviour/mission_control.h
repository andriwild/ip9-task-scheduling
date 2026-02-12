#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <memory>

#include "../model/context.h"
#include "../model/robot_state.h"
#include "../model/event.h"

class HasPendingMission : public BT::ConditionNode {
public:
    HasPendingMission(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        if (ctx->hasPendingMission()) { return BT::NodeStatus::SUCCESS; }
        return BT::NodeStatus::FAILURE;
    }
};

class IsRobotBusy : public BT::ConditionNode {
public:
    IsRobotBusy(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        if (ctx->m_robot->isBusy()) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class AcceptMissionAction : public BT::SyncActionNode {
public:
    AcceptMissionAction(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        if (!ctx->hasPendingMission()) {
             return BT::NodeStatus::FAILURE;
        }

        auto appointment = ctx->popPendingMission();
        ctx->setAppointment(appointment);
        ctx->updateAppointmentState(des::MissionState::IN_PROGRESS);
        const std::string person = appointment->personName;

        assert(ctx->m_employeeLocations.find(person) != ctx->m_employeeLocations.end());
        std::vector<std::string> locations = ctx->m_employeeLocations.at(person);
        assert(!locations.empty());

        ctx->changeRobotState(std::make_unique<SearchState>(locations));
        return BT::NodeStatus::SUCCESS;
    }
};

class RejectMissionAction : public BT::SyncActionNode {
public:
    RejectMissionAction(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        if (!ctx->hasPendingMission()) {
             return BT::NodeStatus::FAILURE;
        }
        
        auto appointment = ctx->popPendingMission();
        appointment->state = des::MissionState::FAILED;
        
        return BT::NodeStatus::SUCCESS;
    }
};

class MissionFeasablityCheck : public BT::SyncActionNode {
public:
    MissionFeasablityCheck(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) 
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        
        auto appointment = ctx->nextPendingMission();
        if (ctx->isMissionFeasible(*appointment, ctx->m_robot->getLocation())){
            return BT::NodeStatus::SUCCESS;
        };
        return BT::NodeStatus::FAILURE;
        
    }
};

