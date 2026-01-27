#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <memory>

#include "../model/context.h"
#include "../model/robot_state.h"

class Docking : public BT::SyncActionNode
{
public:
  Docking(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("ctx"), BT::InputPort<int>("location"),
            BT::InputPort<int>("current_time")};
  }

  BT::NodeStatus tick() override
  {
    auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
    std::string location = config().blackboard->get<std::string>("location");
    int currentTime = config().blackboard.get()->get<int>("current_time");

    if (ctx->m_robot->getLocation() == ctx->m_robot->getIdleLocation()) {
      return BT::NodeStatus::SUCCESS;
    } else {
      ctx->changeRobotState(std::make_unique<MoveState>());
      ctx->scheduleArrival(currentTime, ctx->m_robot->getIdleLocation());
    }
    return BT::NodeStatus::FAILURE;
  }
};

class EnterIdle : public BT::ConditionNode
{
public:
  EnterIdle(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) { }

  static BT::PortsList providedPorts() { return {BT::InputPort<int>("ctx")}; }

  BT::NodeStatus tick() override {
    auto ctx = config().blackboard.get()->get<std::shared_ptr<SimulationContext>>("ctx");
    assert(ctx->m_robot->getLocation() == ctx->m_robot->getIdleLocation());
    ctx->notifyEvent("Robot is idle at " + ctx->m_robot->getLocation());
    ctx->changeRobotState(std::make_unique<IdleState>());
    return BT::NodeStatus::SUCCESS;
  }
};
