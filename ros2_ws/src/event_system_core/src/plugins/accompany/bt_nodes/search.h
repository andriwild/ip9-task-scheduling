#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <algorithm>
#include <deque>
#include <memory>

#include "../../../util/log.h"
#include "engine/contracts/i_sim_context.h"
#include "engine/event/start_drive_event.h"
#include "model/robot.h"
#include "plugins/accompany/accompany_order.h"
#include "plugins/accompany/events/abort_search_event.h"
#include "plugins/accompany/events/scan_point_event.h"
#include "plugins/accompany/events/start_found_person_conversation_event.h"
#include "plugins/accompany/search_exclusion.h"
#include "plugins/accompany/states.h"

namespace des {

// A room without a tour still gets one scan, taken where the robot stands.
inline std::deque<ScanStop> scanRoute(const RoomTour& tour) {
    if (tour.m_path.empty()) {
        return { ScanStop{ Point{}, Polygon{}, false } };
    }
    std::deque<ScanStop> route;
    for (std::size_t i = 0; i < tour.m_path.size(); ++i) {
        route.push_back(ScanStop{ tour.m_path[i], tour.visibilityAt(i), true });
    }
    return route;
}

class IsSearching final : public BT::ConditionNode {
public:
    IsSearching(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const bool isSearching = !ctx->getRobot()->isDriving()
            && dynamic_cast<SearchState*>(ctx->getRobot()->getState()) != nullptr;
        DES_LOG_DEBUG("des.plugin.accompany.search", "IsSearching: %d", isSearching);
        return isSearching ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class FoundPerson final : public BT::ConditionNode {
public:
    FoundPerson(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const bool visible = ctx->getRobot()->isPersonVisible();
        DES_LOG_DEBUG("des.plugin.accompany.search", "FoundPerson: %d", visible);
        return visible ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class StartAccompanyConversation final : public BT::SyncActionNode {
public:
    StartAccompanyConversation(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        ctx->pushEvent(std::make_shared<StartFoundPersonConversationEvent>(ctx->getTime()));
        DES_LOG_DEBUG("des.plugin.accompany.search", "Start Accompany Conversation");
        return BT::NodeStatus::SUCCESS;
    }
};

class HasScanPoint final : public BT::ConditionNode {
public:
    HasScanPoint(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        auto& order = static_cast<AccompanyOrder&>(*ctx->getOrderPtr());
        const std::string room = ctx->getRobot()->getLocation();

        if (isSearchExcluded(*ctx, room)) {
            DES_LOG_DEBUG("des.plugin.accompany.search", "HasScanPoint: room '%s' is excluded from search", room.c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (order.scanRoom != room) {
            order.scanRoom = room;
            order.scanQueue = scanRoute(ctx->room(room).m_tour);
        }
        DES_LOG_DEBUG("des.plugin.accompany.search", "HasScanPoint: room=%s %zu stops left", room.c_str(), order.scanQueue.size());
        if (order.scanQueue.empty()) {
            // Only a fully worked off route justifies claiming the person was absent.
            ctx->getRobot()->markRoomVisitCovered();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class ScanNextPoint final : public BT::SyncActionNode {
public:
    ScanNextPoint(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        // TODO: fix static to dynamic cast
        auto& order = static_cast<AccompanyOrder&>(*ctx->getOrderPtr());

        const ScanStop stop = order.scanQueue.front();
        order.scanQueue.pop_front();
        const auto scan = std::make_shared<ScanPointEvent>(ctx->getTime(), ctx->getOrderPtr());

        DES_LOG_DEBUG("des.plugin.accompany.search", "ScanNextPoint: %zu stops left", order.scanQueue.size());
        if (stop.drive) {
            requestDrive(*ctx, stop.point, stop.visibility, scan);
        } else {
            ctx->startActivity(scan);
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class HasNextLocation final : public BT::ConditionNode {
public:
    HasNextLocation(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const auto& order = static_cast<const AccompanyOrder&>(*ctx->getOrderPtr());
        DES_LOG_DEBUG("des.plugin.accompany.search", "HasNextLocation: %zu locations remaining", order.remainingSearch.size());
        return order.remainingSearch.empty() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
};

class MoveToNextLocation final : public BT::SyncActionNode {
public:
    MoveToNextLocation(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        auto& order = static_cast<AccompanyOrder&>(*ctx->getOrderPtr());

        const std::string nextLocation = order.remainingSearch.front();
        order.remainingSearch.erase(order.remainingSearch.begin());
        order.scanRoom.clear();
        order.scanQueue.clear();
        DES_LOG_DEBUG("des.plugin.accompany.search", "MoveToNextLocation: %s", nextLocation.c_str());
        requestDrive(*ctx, nextLocation);
        return BT::NodeStatus::SUCCESS;
    }
};

class ReportSearchAbort final : public BT::SyncActionNode {
public:
    ReportSearchAbort(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus tick() override {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        ctx->pushEvent(std::make_shared<AbortSearchEvent>(ctx->getTime(), ctx->getOrderPtr()));
        DES_LOG_DEBUG("des.plugin.accompany.search", "Report Search Abort");
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace des
