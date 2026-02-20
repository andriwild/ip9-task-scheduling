#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <memory>

#include "../model/context.h"

class HasPendingMission final : public BT::ConditionNode {
public:
    HasPendingMission(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx        = config().blackboard.get()->get<std::shared_ptr<SimulationContext> >("ctx");
        const bool hasPending = ctx->hasPendingMission();
        RCLCPP_DEBUG(rclcpp::get_logger("BT - MissionControlRoutine"), "HasPendingMission: %d", hasPending);
        if (hasPending) { return BT::NodeStatus::SUCCESS; }
        return BT::NodeStatus::FAILURE;
    }
};

class IsRobotBusy final : public BT::ConditionNode {
public:
    IsRobotBusy(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx    = config().blackboard.get()->get<std::shared_ptr<SimulationContext> >("ctx");
        const bool isBusy = ctx->m_robot->isBusy();
        RCLCPP_DEBUG(rclcpp::get_logger("BT - MissionControlRoutine"), "IsRobotBusy: %d", isBusy);
        if (isBusy) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class IsMissionAssigned final : public BT::ConditionNode {
public:
    IsMissionAssigned(const std::string &name, const BT::NodeConfig &config) : ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx      = config().blackboard.get()->get<std::shared_ptr<SimulationContext> >("ctx");
        const bool assigned = ctx->getAppointment() != nullptr && ctx->getAppointment()->state == des::IN_PROGRESS;
        RCLCPP_DEBUG(rclcpp::get_logger("BT - MissionControlRoutine"), "IsMissionAssigned: %d", assigned);
        if (assigned) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class AcceptMissionAction final : public BT::SyncActionNode {
public:
    AcceptMissionAction(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext> >("ctx");
        assert(ctx->hasPendingMission());
        const auto appointment = ctx->popPendingMission();
        RCLCPP_INFO(rclcpp::get_logger("BT - MissionControlRoutine"), "AcceptMissionAction for: %s",
                    appointment->personName.c_str());
        ctx->setAppointment(appointment);
        ctx->updateAppointmentState(des::MissionState::IN_PROGRESS);
        ctx->m_queue.push(std::make_shared<MissionStartEvent>(ctx->getTime(), appointment));
        return BT::NodeStatus::SUCCESS;
    }
};

class RejectMissionAction final : public BT::SyncActionNode {
public:
    RejectMissionAction(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext> >("ctx");
        RCLCPP_WARN(rclcpp::get_logger("BT - MissionControlRoutine"), "RejectMissionAction");

        assert(ctx->hasPendingMission());
        const auto appointment = ctx->popPendingMission();
        appointment->state     = des::MissionState::REJECTED;
        ctx->m_queue.push(std::make_shared<MissionCompleteEvent>(ctx->getTime(), appointment));
        return BT::NodeStatus::SUCCESS;
    }
};

class MissionFeasibilityCheck final : public BT::SyncActionNode {
public:
    MissionFeasibilityCheck(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext> >("ctx");

        const auto appointment = ctx->nextPendingMission();
        const bool isFeasible  = ctx->isMissionFeasible(*appointment, ctx->m_robot->getLocation());
        RCLCPP_DEBUG(rclcpp::get_logger("BT - MissionControlRoutine"), "MissionFeasibilityCheck: %d", isFeasible);
        if (isFeasible) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};
