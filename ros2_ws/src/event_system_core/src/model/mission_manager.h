#pragma once

#include <cassert>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "../util/types.h"

class MissionManager {
    std::shared_ptr<des::Appointment> m_current = nullptr;
    std::queue<std::shared_ptr<des::Appointment>> m_pending;

public:
    void setCurrent(const std::shared_ptr<des::Appointment>& appointment) {
        m_current = appointment;
    }

    std::shared_ptr<des::Appointment> getCurrent() const {
        return m_current;
    }

    void updateState(const des::MissionState& newState) {
        assert(m_current != nullptr);
        m_current->state = newState;
    }

    void addPending(const std::shared_ptr<des::Appointment>& appointment) {
        m_pending.push(appointment);
        RCLCPP_DEBUG(rclcpp::get_logger("MissionManager"), "Mission added to pending list - queue size: %zu", m_pending.size());
    }

    bool hasPending() const {
        return !m_pending.empty();
    }

    std::shared_ptr<des::Appointment> peekPending() {
        if (m_pending.empty()) { return nullptr; }
        return m_pending.front();
    }

    std::shared_ptr<des::Appointment> popPending() {
        if (m_pending.empty()) { return nullptr; }
        auto appointment = m_pending.front();
        m_pending.pop();
        RCLCPP_DEBUG(rclcpp::get_logger("MissionManager"), "Mission removed from pending list - %zu remaining", m_pending.size());
        return appointment;
    }

    void reset() {
        m_current = nullptr;
        m_pending = std::queue<std::shared_ptr<des::Appointment>>();
    }
};
