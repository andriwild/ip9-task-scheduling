#pragma once

#include <utility>
#include <vector>
#include <memory>
#include <cassert>
#include <optional>
#include <map>

#include "../util/log.h"
#include "engine/event.h"
#include "../sim/i_path_planner.h"
#include "../plugins/order_registry.h"

namespace des {

class Scheduler {
    std::shared_ptr<SimConfig> m_simConfig;
    std::shared_ptr<IPathPlanner> m_plannerNode;
    const RoomMap& m_rooms;

public:

    explicit Scheduler(
        const std::shared_ptr<SimConfig> &simConfig,
        const std::shared_ptr<IPathPlanner> &plannerNode,
        const RoomMap& rooms
    )
        : m_simConfig(simConfig)
        , m_plannerNode(plannerNode)
        , m_rooms(rooms)
    {}

    int timeBuffer() const {
        return m_simConfig->timeBuffer;
    }

    std::vector<std::shared_ptr<MissionDispatchEvent>> createMissionDispatchEvents(OrderList& orders, const std::string& startPos) {
        DES_LOG_DEBUG("des.scheduler", "[SimplePlan] Schedule %zu appointments", orders.size());
        std::vector<std::shared_ptr<MissionDispatchEvent>> events;

        for (const auto& order : orders) {
            auto& plugin = OrderRegistry::instance().get(order->type);
            const int dispatchTime = plugin.planDispatchTime(*order, *this, startPos);
            order->dispatchTime = dispatchTime;
            events.emplace_back(std::make_shared<MissionDispatchEvent>(dispatchTime, order));
        }
        return events;
    }

    // Generic drive-time primitive: uses the standard robotSpeed.
    double robotDriveTime(const std::string& from, const std::string& to) const {
        return getDriveTime(from, to, m_simConfig->robotSpeed);
    }

    // Generic drive-time primitive: caller supplies the speed. Used by plugins
    // that need to compute travel time at a mode-specific speed (e.g. accompany).
    [[nodiscard]] double getDriveTime(const std::string& startPos, const std::string& goalPos, const double speed) const {
        const std::optional<double> dist = m_plannerNode->calcDistance(startPos, goalPos, m_simConfig->cacheEnabled);
        assert(dist.has_value());
        return dist.value() / speed;
    }

    // Time it takes to scan the given room in seconds, same model as the search planner
    [[nodiscard]] double getScanTime(const std::string& room) const {
        auto it = m_rooms.find(room);
        if (it == m_rooms.end() || m_simConfig->robotSpeed <= 0.0) {
            DES_LOG_DEBUG("des.scheduler", "No scan time for '%s', defaulting to 0", room.c_str());
            return 0.0;
        }
        return it->second.m_tour.m_distance / m_simConfig->robotSpeed;
    }
};

}  // namespace des
