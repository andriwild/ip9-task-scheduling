#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <memory>

#include "../model/context.h"
#include "../model/robot_state.h"

class HasPendingMission final : public BT::ConditionNode {
public:
    HasPendingMission(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        if (ctx->hasPendingMission()) { return BT::NodeStatus::SUCCESS; }
        return BT::NodeStatus::FAILURE;
    }
};

class IsRobotBusy final : public BT::ConditionNode {
public:
    IsRobotBusy(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }
    
    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        if (ctx->m_robot->isBusy()) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class AcceptMissionAction final : public BT::SyncActionNode {
public:
    AcceptMissionAction(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        if (!ctx->hasPendingMission()) {
             return BT::NodeStatus::FAILURE;
        }

        const auto appointment = ctx->popPendingMission();
        ctx->setAppointment(appointment);
        ctx->updateAppointmentState(des::MissionState::IN_PROGRESS);
        const std::string person = appointment->personName;

        assert(ctx->m_employeeLocations.contains(person));
        std::vector<std::string> locations = ctx->m_employeeLocations.at(person);
        assert(!locations.empty());

        ctx->changeRobotState(std::make_unique<SearchState>(locations));
        return BT::NodeStatus::SUCCESS;
    }
};

class RejectMissionAction final : public BT::SyncActionNode {
public:
    RejectMissionAction(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        if (!ctx->hasPendingMission()) {
             return BT::NodeStatus::FAILURE;
        }
        
        const auto appointment = ctx->popPendingMission();
        appointment->state = des::MissionState::FAILED;
        
        return BT::NodeStatus::SUCCESS;
    }
};

class MissionFeasibilityCheck final : public BT::SyncActionNode {
public:
    MissionFeasibilityCheck(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        
        const auto appointment = ctx->nextPendingMission();
        if (ctx->isMissionFeasible(*appointment, ctx->m_robot->getLocation())){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

