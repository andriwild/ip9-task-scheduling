#pragma once

#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "../../plugins/i_order.h"
#include "../robot_state.h"

// Single-slot holder for a preemptive interrupt mission (at most one active),
// together with the snapshot of the mission it suspended (for resume on pop).
class InterruptMissionSlot {
    des::OrderPtr m_mission = nullptr;

    // Robot state captured when the interrupt preempted the running mission.
    struct Suspended {
        std::unique_ptr<RobotState> state;
        bool wasDriving = false;
    };
    std::optional<Suspended> m_suspended;

public:
    bool has() const {
        return m_mission != nullptr;
    }

    des::OrderPtr active() const {
        return m_mission;
    }

    // Returns false if the slot is already occupied. `current` is the mission being preempted (logging only).
    bool push(const des::OrderPtr& order, const des::OrderPtr& current) {
        if (m_mission) {
            DES_LOG_WARN(rclcpp::get_logger("des.mission.interrupt"), "Interrupt %d (type=%s) rejected — interrupt %d already active", order->id, order->type.c_str(), m_mission->id);
            return false;
        }
        m_mission = order;
        if (current) {
            DES_LOG_INFO(rclcpp::get_logger("des.mission.interrupt"), "Interrupt %d (type=%s) accepted — preempting mission %d", order->id, order->type.c_str(), current->id);
        } else {
            DES_LOG_INFO(rclcpp::get_logger("des.mission.interrupt"), "Interrupt %d (type=%s) accepted — preempting none", order->id, order->type.c_str());
        }
        return true;
    }

    void pop(const des::OrderPtr& completedOrder) {
        if (m_mission == completedOrder) {
            DES_LOG_INFO(rclcpp::get_logger("des.mission.interrupt"), "Interrupt %d (type=%s) popped", completedOrder->id, completedOrder->type.c_str());
            m_mission = nullptr;
        } else {
            DES_LOG_WARN(rclcpp::get_logger("des.mission.interrupt"), "pop: order %d not active", completedOrder->id);
        }
    }

    // Records the robot state the interrupt suspended, to be restored on takeSuspended().
    void suspend(std::unique_ptr<RobotState> state, bool wasDriving) {
        m_suspended = Suspended{std::move(state), wasDriving};
    }

    // Moves out the suspended snapshot (if any), clearing it.
    std::optional<Suspended> takeSuspended() {
        if (!m_suspended) { return std::nullopt; }
        auto snap = std::move(*m_suspended);
        m_suspended.reset();
        return snap;
    }

    void clear() {
        m_mission = nullptr;
        m_suspended.reset();
    }
};
