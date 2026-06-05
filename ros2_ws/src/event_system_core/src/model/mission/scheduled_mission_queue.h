#pragma once

#include <cstddef>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "../../plugins/i_order.h"

// FIFO queue of dispatched, time-scheduled missions awaiting execution.
class ScheduledMissionQueue {
    std::queue<des::OrderPtr> m_missions;

public:
    void add(const des::OrderPtr& order) {
        m_missions.push(order);
        DES_LOG_DEBUG(rclcpp::get_logger("des.mission.scheduled"), "Mission added to pending list - queue size: %zu", m_missions.size());
    }

    bool has() const {
        return !m_missions.empty();
    }

    des::OrderPtr peek() const {
        if (m_missions.empty()) { return nullptr; }
        return m_missions.front();
    }

    des::OrderPtr pop() {
        if (m_missions.empty()) { return nullptr; }
        auto order = m_missions.front();
        m_missions.pop();
        DES_LOG_DEBUG(rclcpp::get_logger("des.mission.scheduled"), "Mission removed from pending list - %zu remaining", m_missions.size());
        return order;
    }

    std::size_t size() const {
        return m_missions.size();
    }

    void clear() {
        m_missions = std::queue<des::OrderPtr>();
    }
};
