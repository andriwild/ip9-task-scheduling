#pragma once

#include <algorithm>
#include <cassert>
#include <deque>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "../util/types.h"
#include "../model/robot.h"
#include "../model/robot_state.h"
#include "../model/i_sim_context.h"
#include "../plugins/i_order.h"
#include "../plugins/order_registry.h"

// Battery voltage matches Battery::m_voltage; capacity (Ah) * voltage = Wh.
constexpr double kBatteryVoltage = 12.0;
// Reserve added on top of the next-scheduled energy estimate to absorb model
// drift (path-planner variance, battery curve).
constexpr double kBackgroundEnergySafetyMarginWh = 5.0;

class MissionManager {
    des::OrderPtr m_current = nullptr;
    std::queue<des::OrderPtr> m_pending;

    // Background tasks ("egal wann"): dispatched opportunistically when no
    // scheduled mission is pending. Populated once at scenario load.
    std::vector<des::OrderPtr> m_background;

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

    void addBackground(const des::OrderPtr order) {
        m_background.push_back(order);
        RCLCPP_DEBUG(rclcpp::get_logger("MissionManager"), "Background mission added - list size: %zu", m_background.size());
    }

    bool hasBackground() const {
        return !m_background.empty();
    }

    des::OrderPtr peekBackground() {
        if (m_background.empty()) { return nullptr; }
        return m_background.front();
    }

    des::OrderPtr popBackground() {
        if (m_background.empty()) { return nullptr; }
        auto order = m_background.front();
        m_background.erase(m_background.begin());
        RCLCPP_DEBUG(rclcpp::get_logger("MissionManager"), "Background mission popped - %zu remaining", m_background.size());
        return order;
    }

    bool removeBackground(int orderId) {
        auto it = std::find_if(m_background.begin(), m_background.end(),
                               [orderId](const des::OrderPtr& o) { return o && o->id == orderId; });
        if (it == m_background.end()) { return false; }
        m_background.erase(it);
        RCLCPP_DEBUG(rclcpp::get_logger("MissionManager"), "Background mission %d removed - %zu remaining", orderId, m_background.size());
        return true;
    }

    const std::vector<des::OrderPtr>& background() const {
        return m_background;
    }

    // Pick the first background mission that is feasible — either directly
    // (enough battery to do BG and still cover the next scheduled mission)
    // or via the charge-time fallback (the gap between BG completion and the
    // next scheduled dispatch is long enough to recharge the deficit).
    // Removes the chosen mission from the queue and marks it as current.
    //
    // Selection strategy is intentionally simple (first-feasible); swap in
    // closest / oldest / best-ratio here when more sophistication is needed.
    des::OrderPtr acceptFeasibleBackground(const ISimContext& ctx,
                                           double safetyMarginWh = kBackgroundEnergySafetyMarginWh) {
        if (m_background.empty()) { return nullptr; }

        const auto robot       = ctx.getRobot();
        const auto batStats    = robot->m_bat->getStats();
        const double currentWh = batStats.soc * batStats.capacity * kBatteryVoltage;
        const double capacityWh = batStats.capacity * kBatteryVoltage;
        const std::string& startLoc = robot->getLocation();
        const auto cfg = ctx.getConfig();
        const std::string& dockLoc = cfg->dockLocation;
        const int now              = ctx.getTime();
        const double netChargeW    = cfg->chargingRate - cfg->energyConsumptionBase;

        // Reserve for next scheduled (assume robot dock-starts that mission).
        double reservedWh = 0.0;
        auto next = ctx.peekNextScheduledOrder();
        if (next) {
            const auto& nextPlugin = OrderRegistry::instance().get(next->type);
            reservedWh = nextPlugin.estimateMissionEnergy(*next, ctx, dockLoc);
        }
        const double requiredWh = reservedWh + safetyMarginWh;
        const auto nextDispatch = ctx.getNextScheduledDispatchTime();
        const auto simEndTime   = ctx.getSimulationEndTime();

        for (auto it = m_background.begin(); it != m_background.end(); ++it) {
            const auto& candidate = *it;
            const auto& plugin    = OrderRegistry::instance().get(candidate->type);
            const double bgEnergy   = plugin.estimateMissionEnergy(*candidate, ctx, startLoc);
            const double bgDuration = plugin.estimateMissionDuration(*candidate, ctx, startLoc);
            const double postBgWh   = currentWh - bgEnergy;
            const double finishTime = now + bgDuration;

            // Time bound: a mission that wouldn't complete before the sim
            // terminates should never start.
            if (simEndTime && finishTime > *simEndTime) {
                continue;
            }

            // Hard battery floor: the BG itself must not drain the battery
            // below the safety margin, even when a long charge window would
            // make the next-scheduled budget recoverable. Without this the
            // charge-fallback path would happily accept missions that deplete
            // the battery mid-execution.
            if (postBgWh < safetyMarginWh) {
                continue;
            }

            bool feasible = false;
            const char* mode = "";

            if (postBgWh >= requiredWh) {
                feasible = true;
                mode     = "direct";
            } else if (nextDispatch && netChargeW > 0.0) {
                // Charge-time fallback: how long does the robot have at the
                // dock between BG completion and the next scheduled dispatch?
                const double chargeWindow = static_cast<double>(*nextDispatch) - finishTime;
                // Cap recovery target at full capacity (can't store more).
                const double recoverTarget = std::min(requiredWh, capacityWh);
                const double deficitWh     = recoverTarget - postBgWh;
                const double chargeNeeded  = (deficitWh * 3600.0) / netChargeW;
                if (chargeWindow >= chargeNeeded) {
                    feasible = true;
                    mode     = "charge-fallback";
                }
            }

            if (feasible) {
                RCLCPP_INFO(rclcpp::get_logger("MissionManager"),
                            "Accept background %d (type=%s, est=%.1fWh, reserve=%.1fWh, battery=%.1fWh, mode=%s)",
                            candidate->id, candidate->type.c_str(), bgEnergy, reservedWh, currentWh, mode);
                auto accepted = candidate;
                m_background.erase(it);
                m_current = accepted;
                return accepted;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("MissionManager"),
                    "No feasible background mission (battery=%.1fWh, reserved=%.1fWh, safety=%.1fWh)",
                    currentWh, reservedWh, safetyMarginWh);
        return nullptr;
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
        m_background = std::vector<des::OrderPtr>();
        m_interrupts.clear();
        m_suspendedOrder = nullptr;
        m_suspendedState.reset();
    }
};
