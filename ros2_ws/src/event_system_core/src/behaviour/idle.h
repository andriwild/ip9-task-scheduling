#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <memory>

#include "../model/context.h"
#include "../model/robot_state.h"

class IsIdle: public BT::ConditionNode {
public:
    IsIdle(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        if (ctx->m_robot->getStateType() == des::RobotStateType::IDLE) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};



class Docking : public BT::SyncActionNode {
public:
    Docking(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("ctx") };
    }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");

        if (ctx->m_robot->getLocation() == ctx->m_robot->getIdleLocation()) {
            return BT::NodeStatus::SUCCESS;
        }
        ctx->m_queue.push(std::make_shared<StartDriveEvent>(ctx->getTime(), ctx->m_robot->getIdleLocation()));
        return BT::NodeStatus::FAILURE;
    }
};

class EnterIdle : public BT::ConditionNode {
public:
    EnterIdle(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) { }

    static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

    BT::NodeStatus tick() override {
        auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
        assert(ctx->m_robot->getLocation() == ctx->m_robot->getIdleLocation());
        ctx->changeRobotState(std::make_unique<IdleState>());
        return BT::NodeStatus::SUCCESS;
    }
};
