#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include "../../../util/log.h"
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>

#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "engine/event/start_drive_event.h"
#include "plugins/clean/clean_order.h"
#include "plugins/clean/clean_plugin.h"
#include "plugins/clean/events/start_clean_event.h"
#include "plugins/clean/events/end_clean_event.h"
#include "model/room.h"

namespace des {

class CleanIsAtTargetLocation final : public BT::ConditionNode {
public:
    CleanIsAtTargetLocation(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const auto& order = dynamic_cast<CleanOrder&>(*ctx->getOrderPtr());
        const auto robot = ctx->getRobot();
        const bool atTarget = robot->getLocation() == order.roomName && !robot->isDriving();
        DES_LOG_DEBUG("des.plugin.clean", "CleanIsAtTargetLocation: %d", atTarget);
        return atTarget ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class CleanGoToLocation final : public BT::StatefulActionNode {
public:
    CleanGoToLocation(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus onStart() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        if (ctx->getRobot()->isDriving()) {
            return BT::NodeStatus::RUNNING;  // resumed mid-drive after interrupt
        }
        const auto& order = static_cast<CleanOrder&>(*ctx->getOrderPtr());
        DES_LOG_DEBUG("des.plugin.clean", "CleanGoToLocation: -> %s", order.roomName.c_str());
        requestDrive(*ctx, order.roomName);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const auto& order = static_cast<CleanOrder&>(*ctx->getOrderPtr());
        const auto robot = ctx->getRobot();
        if (robot->getLocation() == order.roomName && !robot->isDriving()) {
            DES_LOG_DEBUG("des.plugin.clean", "CleanGoToLocation: arrived at %s", order.roomName.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}
};

class ExecuteClean final : public BT::StatefulActionNode {
public:
    ExecuteClean(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus onStart() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const auto order = ctx->getOrderPtr();
        // Idempotent on resume after interrupt: only push StartCleanEvent if mission hasn't been started yet.
        if (order && order->state == OrderState::PENDING) {
            DES_LOG_DEBUG("des.plugin.clean", "ExecuteClean: start");
            ctx->pushEvent(std::make_shared<StartCleanEvent>(ctx->getTime(), order));
            return BT::NodeStatus::RUNNING;
        }
        // Resumed after an interrupt: continue the sweep now instead of waiting for the next tick.
        return onRunning();
    }

    BT::NodeStatus onRunning() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const auto order = ctx->getOrderPtr();
        if (!order || order->state == OrderState::COMPLETED) {
            DES_LOG_DEBUG("des.plugin.clean", "ExecuteClean: done");
            ctx->getRobot()->setServicing(false);
            return BT::NodeStatus::SUCCESS;
        }
        const auto robot = ctx->getRobot();
        if (order->state != OrderState::IN_PROGRESS || robot->isDriving()) {
            return BT::NodeStatus::RUNNING;
        }

        auto& cleanOrder = static_cast<CleanOrder&>(*order);
        const Room& room = ctx->room(cleanOrder.roomName);
        const RoomTour& tour = room.m_tour;

        const double duration = cleanDurationSeconds(room.m_area.value_or(1.0), ctx->getConfig()->robotSpeed);
        const double sweepDistance = tour.m_distance;

        robot->setServicing(true);

        if (sweepDistance <= 0.0) {
            DES_LOG_DEBUG("des.plugin.clean", "ExecuteClean: %s has no drivable tour, cleaning in place", cleanOrder.roomName.c_str());
            robot->setServicing(false);
            robot->markRoomVisitCovered();
            ctx->startActivity(std::make_shared<EndCleanEvent>(ctx->getTime() + static_cast<int>(duration), order));
            return BT::NodeStatus::RUNNING;
        }

        if (cleanOrder.sweepIndex >= tour.m_path.size()) {
            DES_LOG_DEBUG("des.plugin.clean", "ExecuteClean: swept %zu points of %s", tour.m_path.size(), cleanOrder.roomName.c_str());
            robot->setServicing(false);
            robot->markRoomVisitCovered();
            ctx->startActivity(std::make_shared<EndCleanEvent>(ctx->getTime(), order));
            return BT::NodeStatus::RUNNING;
        }

        const double sweepSpeed = sweepDistance / duration;
        const std::size_t point = cleanOrder.sweepIndex;
        cleanOrder.sweepIndex++;
        requestDrive(*ctx, tour.m_path[point], tour.visibilityAt(point), nullptr, sweepSpeed);
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}
};

}  // namespace des
