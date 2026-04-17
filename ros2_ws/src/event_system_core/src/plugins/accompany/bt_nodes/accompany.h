#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>

#include "../../../model/event.h"
#include "../../../model/i_sim_context.h"
#include "../../../model/robot_state.h"
#include "../../../model/robot.h"


class IsAccompany final : public BT::ConditionNode {
public:
    IsAccompany(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        const bool isArrived = ctx->getRobot()->getLocation() == ctx->getRobot()->getTargetLocation();
        const bool isAccompany = ctx->getRobot()->getStateType() == des::RobotStateType::ACCOMPANY;
        RCLCPP_DEBUG(rclcpp::get_logger("BT - AccompanyRoutine"), "IsAccompany: %d (arrived: %d)", isAccompany, isArrived);
        if (isAccompany && isArrived) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};
 
class ArrivedWithPerson final : public BT::ConditionNode {
public:
    ArrivedWithPerson(const std::string& name, const BT::NodeConfig& config): ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");
        // TODO: add randomness
        const bool arrived = true;
        RCLCPP_DEBUG(rclcpp::get_logger("BT - AccompanyRoutine"), "ArrivedWithPerson: %d", arrived);
        return arrived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class StartDropOffConversation final : public BT::SyncActionNode {
public:
    StartDropOffConversation(const std::string& name, const BT::NodeConfig& config): SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx"), BT::InputPort<int>("current_time") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        ctx->pushEvent(std::make_shared<StartDropOffConversationEvent>(ctx->getTime()));
        ctx->changeRobotState(std::make_unique<ConversateState>());
        RCLCPP_INFO(rclcpp::get_logger("BT - AccompanyRoutine"), "Start Drop-off Conversation");
        return BT::NodeStatus::SUCCESS;
    }
};

class AbortAccompany final : public BT::SyncActionNode {
public:
    AbortAccompany(const std::string& name, const BT::NodeConfig& config): SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<std::shared_ptr<ISimContext>>("ctx");

        ctx->updateOrderState(des::MissionState::FAILED);
        ctx->changeRobotState(std::make_unique<IdleState>());
        RCLCPP_WARN(rclcpp::get_logger("BT - AccompanyRoutine"), "Abort Accompany!");
        return BT::NodeStatus::SUCCESS;
    }
};
