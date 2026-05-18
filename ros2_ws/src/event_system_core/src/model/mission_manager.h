#pragma once

#include <algorithm>
#include <cassert>
#include <deque>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "../util/types.h"
#include "../model/robot_state.h"
#include "../plugins/i_order.h"

class MissionManager {
    des::OrderPtr m_current = nullptr;
    std::queue<des::OrderPtr> m_pending;

    // Active interrupts in chronological push order (back = most recent = current).
    // Deque (not stack) so we can remove mid-list completions when an interrupt with shorter
    // duration ends before one above it.
    std::deque<des::OrderPtr> m_interrupts;

    // Snapshot of the mission displaced by the first interrupt push.
    // Set only on the [empty -> non-empty] transition, restored on [non-empty -> empty].
    des::OrderPtr m_suspendedOrder = nullptr;
    std::unique_ptr<RobotState> m_suspendedState;

public:
    void setCurrent(const des::OrderPtr order) {
        m_current = order;
    }

    des::OrderPtr getCurrent() const {
        return m_current;
    }

    void updateState(const des::MissionState& newState) {
        assert(m_current != nullptr);
        m_current->state = newState;
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

    bool hasInterrupt() const {
        return !m_interrupts.empty();
    }

    void pushInterrupt(des::OrderPtr order, std::unique_ptr<RobotState> state) {
        if (m_interrupts.empty()) {
            m_suspendedOrder = m_current;
            m_suspendedState = std::move(state);
        }
        // Nested pushes skip the state snapshot — interrupts don't change RobotState,
        // so the snapshot from the first push is still valid.
        m_interrupts.push_back(order);
        m_current = std::move(order);
    }

    // Removes completedOrder from the interrupt list. Returns the RobotState to restore
    // if this was the LAST active interrupt (list now empty); nullptr otherwise.
    std::unique_ptr<RobotState> popInterrupt(const des::OrderPtr& completedOrder) {
        auto it = std::find(m_interrupts.begin(), m_interrupts.end(), completedOrder);
        if (it != m_interrupts.end()) {
            m_interrupts.erase(it);
        }

        if (m_interrupts.empty()) {
            // Don't restore a suspended order that completed mid-interrupt.
            if (m_suspendedOrder && m_suspendedOrder->state != des::MissionState::COMPLETED) {
                m_current = m_suspendedOrder;
            } else {
                m_current = nullptr;
            }
            m_suspendedOrder = nullptr;
            return std::move(m_suspendedState);
        }
        m_current = m_interrupts.back();
        return nullptr;
    }

    void reset() {
        m_current = nullptr;
        m_pending = std::queue<des::OrderPtr>();
        m_interrupts.clear();
        m_suspendedOrder = nullptr;
        m_suspendedState.reset();
    }
};
