#pragma once

#include <algorithm>
#include <cassert>
#include <deque>
#include <memory>
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "../util/types.h"
#include "../model/battery.hpp"
#include "../model/robot.h"
#include "../model/robot_state.h"
#include "../model/i_sim_context.h"
#include "../plugins/i_order.h"
#include "../plugins/order_registry.h"

// Headroom on top of the next-scheduled estimate — absorbs model drift.
constexpr double kBackgroundEnergySafetyMarginWh = 5.0;

class MissionManager {
    des::OrderPtr m_current = nullptr;
    std::queue<des::OrderPtr> m_pending;
    std::vector<des::OrderPtr> m_background;
    std::deque<des::OrderPtr> m_interrupts;

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

    // First-feasible background mission. Two acceptance paths:
    //   direct          — post-BG battery covers next-scheduled reserve.
    //   charge-fallback — dock window before next dispatch refills the deficit.
    //
    //   TODO: refactor
    des::OrderPtr acceptFeasibleBackground(
        const ISimContext& ctx,
        double safetyMarginWh = kBackgroundEnergySafetyMarginWh
    ) {
        if (m_background.empty()) { return nullptr; }

        const auto robot            = ctx.getRobot();
        const auto batStats         = robot->m_bat->getStats();
        const double currentWh      = batStats.soc * batStats.capacity * Battery::kVoltage;
        const double capacityWh     = batStats.capacity * Battery::kVoltage;
        const auto cfg              = ctx.getConfig();
        const std::string& startLoc = robot->getLocation();
        const std::string& dockLoc  = cfg->dockLocation;
        const int now               = ctx.getTime();
        const double netChargeW     = cfg->chargingRate - cfg->energyConsumptionBase;

        double reservedWh = 0.0;
        if (auto next = ctx.peekNextScheduledOrder()) {
            const auto& nextPlugin = OrderRegistry::instance().get(next->type);
            reservedWh = nextPlugin.estimateMissionEnergy(*next, ctx, dockLoc);
        }
        const double requiredWh = reservedWh + safetyMarginWh;
        const auto nextDispatch  = ctx.getNextScheduledDispatchTime();
        const auto simEndTime    = ctx.getSimulationEndTime();

        struct Verdict { const char* mode; double bgEnergy; double bgDuration; };

        auto evaluate = [&](const des::OrderPtr& candidate) -> std::optional<Verdict> {
            const auto& plugin      = OrderRegistry::instance().get(candidate->type);
            const double bgEnergy   = plugin.estimateMissionEnergy(*candidate, ctx, startLoc);
            const double bgDuration = plugin.estimateMissionDuration(*candidate, ctx, startLoc);
            const double postBgWh   = currentWh - bgEnergy;
            const double finishTime = now + bgDuration;

            const bool fitsSimWindow = !simEndTime || finishTime <= *simEndTime;
            const bool fitsDispatch  = !nextDispatch || finishTime <= *nextDispatch;
            const bool aboveFloor    = postBgWh >= safetyMarginWh;

            if (!fitsSimWindow || !fitsDispatch || !aboveFloor) { return std::nullopt; }

            if (postBgWh >= requiredWh) { return Verdict{"direct", bgEnergy, bgDuration}; }

            if (nextDispatch && netChargeW > 0.0) {
                const double chargeWindow  = static_cast<double>(*nextDispatch) - finishTime;
                const double recoverTarget = std::min(requiredWh, capacityWh);
                const double deficitWh     = recoverTarget - postBgWh;
                const double chargeNeeded  = (deficitWh * 3600.0) / netChargeW;
                if (chargeWindow >= chargeNeeded) { return Verdict{"charge-fallback", bgEnergy, bgDuration}; }
            }
            return std::nullopt;
        };

        std::optional<Verdict> verdict;
        auto it = std::find_if(m_background.begin(), m_background.end(),
            [&](const des::OrderPtr& c) {
                verdict = evaluate(c);
                return verdict.has_value();
            });

        if (it == m_background.end()) {
            RCLCPP_INFO(rclcpp::get_logger("MissionManager"),
                        "No feasible background mission (battery=%.1fWh, reserved=%.1fWh, safety=%.1fWh)",
                        currentWh, reservedWh, safetyMarginWh);
            return nullptr;
        }

        const auto accepted = *it;
        RCLCPP_INFO(rclcpp::get_logger("MissionManager"),
                    "Accept background %d (type=%s, est=%.1fWh, dur=%.0fs, reserve=%.1fWh, battery=%.1fWh, mode=%s)",
                    accepted->id, accepted->type.c_str(),
                    verdict->bgEnergy, verdict->bgDuration, reservedWh, currentWh, verdict->mode);
        m_background.erase(it);
        m_current = accepted;
        return accepted;
    }

    bool hasInterrupt() const {
        return !m_interrupts.empty();
    }

    // The previously running mission stays in m_current; the BT-routes via
    // hasInterrupt() to the interrupt subtree without overwriting m_current.
    void pushInterrupt(const des::OrderPtr& order) {
        m_interrupts.push_back(order);
    }

    // Returns true if this was the last interrupt on the stack.
    bool popInterrupt(const des::OrderPtr& completedOrder) {
        auto it = std::find(m_interrupts.begin(), m_interrupts.end(), completedOrder);
        if (it != m_interrupts.end()) {
            m_interrupts.erase(it);
        }
        return m_interrupts.empty();
    }

    des::OrderPtr activeInterrupt() const {
        return m_interrupts.empty() ? nullptr : m_interrupts.back();
    }

    void reset() {
        m_current = nullptr;
        m_pending = std::queue<des::OrderPtr>();
        m_background = std::vector<des::OrderPtr>();
        m_interrupts.clear();
    }
};
