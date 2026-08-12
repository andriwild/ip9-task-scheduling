#pragma once

#include <cstddef>
#include <queue>

#include "util/log.h"
#include "model/order.h"

namespace des {

// FIFO queue of dispatched, time-scheduled missions awaiting execution.
class ScheduledOrderQueue {
    std::queue<OrderPtr> m_missions;

public:
    void add(const OrderPtr& order) {
        m_missions.push(order);
        DES_LOG_DEBUG("des.mission.scheduled", "Mission added to pending list - queue size: %zu", m_missions.size());
    }

    bool has() const {
        return !m_missions.empty();
    }

    OrderPtr peek() const {
        if (m_missions.empty()) { return nullptr; }
        return m_missions.front();
    }

    OrderPtr pop() {
        if (m_missions.empty()) { return nullptr; }
        auto order = m_missions.front();
        m_missions.pop();
        DES_LOG_DEBUG("des.mission.scheduled", "Mission removed from pending list - %zu remaining", m_missions.size());
        return order;
    }

    std::size_t size() const {
        return m_missions.size();
    }

    void clear() {
        m_missions = std::queue<OrderPtr>();
    }
};

}  // namespace des
