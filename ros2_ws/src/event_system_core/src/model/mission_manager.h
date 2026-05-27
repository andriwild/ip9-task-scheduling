#pragma once

#include <algorithm>
#include <cassert>
#include <memory>
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "../util/log.h"
#include "../util/types.h"
#include "../model/battery.hpp"
#include "../model/robot.h"
#include "../model/robot_state.h"
#include "../model/i_sim_context.h"
#include "../plugins/i_order.h"
#include "../plugins/order_registry.h"

constexpr double kBackgroundEnergySafetyMarginWh = 5.0;

class MissionManager {
    des::OrderPtr m_current = nullptr;
    std::queue<des::OrderPtr> m_pending; // scheduled mission dispatches
    std::vector<des::OrderPtr> m_background;
    des::OrderPtr m_interrupt = nullptr;

public:
    void setCurrent(const des::OrderPtr order) {
        if (order) {
            DES_LOG_DEBUG(rclcpp::get_logger("des.mission_manager"), "Current mission set: %d (type=%s)", order->id, order->type.c_str());
        } else if (m_current) {
            DES_LOG_DEBUG(rclcpp::get_logger("des.mission_manager"), "Current mission cleared (was %d)", m_current->id);
        }
        m_current = order;
    }

    des::OrderPtr getCurrent() const {
        return m_current;
    }

    void updateState(const des::MissionState& newState) {
        assert(m_current != nullptr);
        DES_LOG_INFO(rclcpp::get_logger("des.mission_manager"),
                     "Mission %d (type=%s) state: %s -> %s",
                     m_current->id, m_current->type.c_str(),
                     des::missionStateStr(m_current->state).c_str(),
                     des::missionStateStr(newState).c_str());
        m_current->state = newState;
    }

    void addPending(const des::OrderPtr order) {
        m_pending.push(order);
        DES_LOG_DEBUG(rclcpp::get_logger("des.mission_manager"), "Mission added to pending list - queue size: %zu", m_pending.size());
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
        DES_LOG_DEBUG(rclcpp::get_logger("des.mission_manager"), "Mission removed from pending list - %zu remaining", m_pending.size());
        return appointment;
    }

    void addBackground(const des::OrderPtr order) {
        m_background.push_back(order);
        DES_LOG_DEBUG(rclcpp::get_logger("des.mission_manager"), "Background mission added - list size: %zu", m_background.size());
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
        DES_LOG_DEBUG(rclcpp::get_logger("des.mission_manager"), "Background mission popped - %zu remaining", m_background.size());
        return order;
    }

    bool removeBackground(int orderId) {
        auto it = std::find_if(m_background.begin(), m_background.end(),
                               [orderId](const des::OrderPtr& o) { return o && o->id == orderId; });
        if (it == m_background.end()) { return false; }
        m_background.erase(it);
        DES_LOG_DEBUG(rclcpp::get_logger("des.mission_manager"), "Background mission %d removed - %zu remaining", orderId, m_background.size());
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

        // get estimated energy of the next scheduled mission
        // this energy must be available after a bg mission is executed
        double reservedWh = 0.0;
        if (auto next = ctx.peekNextScheduledOrder()) {
            const auto& nextPlugin = OrderRegistry::instance().get(next->type);
            reservedWh = nextPlugin.estimateMissionEnergy(*next, ctx, dockLoc);
        }

        const double requiredWh  = reservedWh + safetyMarginWh;
        const auto nextDispatch  = ctx.getNextScheduledDispatchTime();
        const auto simEndTime    = ctx.getSimulationEndTime();

        struct Result { const char* mode; double bgEnergy; double bgDuration; };

        auto evaluate = [&](const des::OrderPtr& candidate) -> std::optional<Result> {
            const auto& plugin      = OrderRegistry::instance().get(candidate->type);
            const double bgEnergy   = plugin.estimateMissionEnergy(*candidate, ctx, startLoc);
            const double bgDuration = plugin.estimateMissionDuration(*candidate, ctx, startLoc);
            
            // remaining energy and time after executing bg mission
            const double postBgWh   = currentWh - bgEnergy;
            const double finishTime = now + bgDuration;

            // check timing constraints (fits the mission into the timing slot)
            const bool fitsSimWindow = !simEndTime || finishTime <= *simEndTime;
            const bool fitsDispatch  = !nextDispatch || finishTime <= *nextDispatch;
            const bool aboveFloor    = postBgWh >= safetyMarginWh;

            if (!fitsSimWindow || !fitsDispatch || !aboveFloor) { 
                return std::nullopt; 
            }

            if (postBgWh >= requiredWh) { 
                return Result{"direct", bgEnergy, bgDuration}; 
            }

            // after executing the bg mission, check if enough time is available to refill the battery before scheduled mission
            if (nextDispatch && netChargeW > 0.0) {
                const double chargeWindow  = static_cast<double>(*nextDispatch) - finishTime;
                const double recoverTarget = std::min(requiredWh, capacityWh);
                const double deficitWh     = recoverTarget - postBgWh;
                const double chargeNeeded  = (deficitWh * 3600.0) / netChargeW;
                if (chargeWindow >= chargeNeeded) { return Result{"charge-fallback", bgEnergy, bgDuration}; }
            }
            return std::nullopt;
        };

        // simple approach to find first feasible bg mission
        std::optional<Result> result;
        auto it = std::find_if(m_background.begin(), m_background.end(),
            [&](const des::OrderPtr& c) {
                result = evaluate(c);
                return result.has_value();
            });

        if (it == m_background.end()) {
            DES_LOG_INFO(rclcpp::get_logger("des.mission_manager"),
                        "No feasible background mission (battery=%.1fWh, reserved=%.1fWh, safety=%.1fWh)",
                        currentWh, reservedWh, safetyMarginWh);
            return nullptr;
        }

        const auto accepted = *it;
        DES_LOG_INFO(rclcpp::get_logger("des.mission_manager"),
                    "Accept background %d (type=%s, est=%.1fWh, dur=%.0fs, reserve=%.1fWh, battery=%.1fWh, mode=%s)",
                    accepted->id, accepted->type.c_str(),
                    result->bgEnergy, result->bgDuration, reservedWh, currentWh, result->mode);
        m_background.erase(it);
        m_current = accepted;
        return accepted;
    }

    bool hasInterrupt() const {
        return m_interrupt != nullptr;
    }

    bool pushInterrupt(const des::OrderPtr& order) {
        if (m_interrupt) {
            DES_LOG_WARN(rclcpp::get_logger("des.mission_manager"), "Interrupt %d (type=%s) rejected — interrupt %d already active", order->id, order->type.c_str(), m_interrupt->id);
            return false;
        }
        m_interrupt = order;
        if (m_current) {
            DES_LOG_INFO(rclcpp::get_logger("des.mission_manager"), "Interrupt %d (type=%s) accepted — preempting mission %d", order->id, order->type.c_str(), m_current->id);
        } else {
            DES_LOG_INFO(rclcpp::get_logger("des.mission_manager"), "Interrupt %d (type=%s) accepted — preempting none", order->id, order->type.c_str());
        }
        return true;
    }

    void popInterrupt(const des::OrderPtr& completedOrder) {
        if (m_interrupt == completedOrder) {
            DES_LOG_INFO(rclcpp::get_logger("des.mission_manager"), "Interrupt %d (type=%s) popped", completedOrder->id, completedOrder->type.c_str());
            m_interrupt = nullptr;
        } else {
            DES_LOG_WARN(rclcpp::get_logger("des.mission_manager"), "popInterrupt: order %d not active", completedOrder->id); }
    }

    des::OrderPtr activeInterrupt() const {
        return m_interrupt;
    }

    void reset() {
        DES_LOG_INFO(rclcpp::get_logger("des.mission_manager"),
                     "Reset (pending=%zu, background=%zu, interrupt=%s cleared)",
                     m_pending.size(), m_background.size(), m_interrupt ? "yes" : "no");
        m_current = nullptr;
        m_pending = std::queue<des::OrderPtr>();
        m_background = std::vector<des::OrderPtr>();
        m_interrupt = nullptr;
    }
};
