#pragma once

#include <optional>
#include <string>

#include "util/log.h"
#include "model/order.h"

namespace des {

// Single-slot holder for a preemptive interrupt mission (at most one active),
// remembering whether the preempted mission was mid-drive so it can resume on pop.
class InterruptMissionSlot {
    OrderPtr m_mission = nullptr;

    struct Suspended {
        bool wasDriving = false;
    };
    std::optional<Suspended> m_suspended;

public:
    bool has() const {
        return m_mission != nullptr;
    }

    OrderPtr active() const {
        return m_mission;
    }

    // Returns false if the slot is already occupied. `current` is the mission being preempted (logging only).
    bool push(const OrderPtr& order, const OrderPtr& current) {
        if (m_mission) {
            DES_LOG_DEBUG("des.mission.interrupt", "Interrupt %d (type=%s) rejected — interrupt %d already active", order->id, order->type.c_str(), m_mission->id);
            return false;
        }
        m_mission = order;
        if (current) {
            DES_LOG_DEBUG("des.mission.interrupt", "Interrupt %d (type=%s) accepted — preempting mission %d", order->id, order->type.c_str(), current->id);
        } else {
            DES_LOG_DEBUG("des.mission.interrupt", "Interrupt %d (type=%s) accepted — preempting none", order->id, order->type.c_str());
        }
        return true;
    }

    void pop(const OrderPtr& completedOrder) {
        if (m_mission == completedOrder) {
            DES_LOG_DEBUG("des.mission.interrupt", "Interrupt %d (type=%s) popped", completedOrder->id, completedOrder->type.c_str());
            m_mission = nullptr;
        } else {
            const std::string activeId = m_mission ? std::to_string(m_mission->id) : "none";
            DES_LOG_WARN("des.mission.interrupt", "Interrupt %d (type=%s) completed but is not the active interrupt (active=%s) — already popped or superseded, ignoring", completedOrder->id, completedOrder->type.c_str(), activeId.c_str());
        }
    }

    void suspend(bool wasDriving) {
        m_suspended = Suspended{wasDriving};
    }

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

}  // namespace des
