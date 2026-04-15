#pragma once

#include <cassert>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "../util/types.h"
#include "../plugins/i_order.h"

class MissionManager {
    des::OrderPtr m_currentOrder = nullptr;
    std::queue<des::OrderPtr> m_pending;

public:
    void setCurrent(const des::OrderPtr order) {
        m_currentOrder = order;
    }

    des::OrderPtr getCurrent() const {
        return m_currentOrder;
    }

    void updateState(const des::MissionState& newState) {
        assert(m_currentOrder != nullptr);
        m_currentOrder->state = newState;
    }

    void addPending(const des::OrderPtr order) {
        m_pending.push(order);
        RCLCPP_DEBUG(rclcpp::get_logger("MissionManager"), "Mission added to pending list - queue size: %zu", m_pending.size());
    }

    bool hasPending() const {
        return !m_pending.empty();
    }

    des::OrderPtr peekPending() {
        if (m_pending.empty()) { return nullptr; }
        return m_pending.front();
    }

    des::OrderPtr popPending() {
        if (m_pending.empty()) { return nullptr; }
        auto appointment = m_pending.front();
        m_pending.pop();
        RCLCPP_DEBUG(rclcpp::get_logger("MissionManager"), "Mission removed from pending list - %zu remaining", m_pending.size());
        return appointment;
    }

    void reset() {
        m_currentOrder = nullptr;
        m_pending = std::queue<des::OrderPtr>();
    }
};
