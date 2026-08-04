#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <memory>

#include "../../../util/log.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"
#include "plugins/accompany/accompany_order.h"
#include "plugins/accompany/events/start_drop_off_conversation_event.h"
#include "plugins/accompany/states.h"

class IsAccompany final : public BT::ConditionNode {
public:
    IsAccompany(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const bool isAccompany = dynamic_cast<AccompanyState*>(ctx->getRobot()->getState()) != nullptr;
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.accompany"), "IsAccompany: %d", isAccompany);
        return isAccompany ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class HasArrived final : public BT::ConditionNode {
public:
    HasArrived(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const auto robot = ctx->getRobot();
        const bool arrived = !robot->isDriving() && robot->getLocation() == robot->getTargetLocation();
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.accompany"), "HasArrived: %d", arrived);
        return arrived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class ArrivedWithPerson final : public BT::ConditionNode {
public:
    ArrivedWithPerson(const std::string& name, const BT::NodeConfig& config): ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const auto& order = static_cast<const AccompanyOrder&>(*ctx->getOrderPtr());
        const bool arrived = ctx->getRobot()->getLocation() == order.roomName;
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.accompany"), "ArrivedWithPerson: %d", arrived);
        return arrived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class StartDropOffConversation final : public BT::SyncActionNode {
public:
    StartDropOffConversation(const std::string& name, const BT::NodeConfig& config): SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        ctx->pushEvent(std::make_shared<StartDropOffConversationEvent>(ctx->getTime()));
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.accompany"), "Start Drop-off Conversation");
        return BT::NodeStatus::SUCCESS;
    }
};
