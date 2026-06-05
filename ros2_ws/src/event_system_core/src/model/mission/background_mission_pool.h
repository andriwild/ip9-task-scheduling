#pragma once

#include <algorithm>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "../../plugins/i_order.h"
#include "../../plugins/order_registry.h"
#include "../battery.hpp"
#include "../robot.h"
#include "../i_sim_context.h"

constexpr double kBackgroundEnergySafetyMarginWh = 5.0;

// Pool of opportunistic background missions executed to fill idle time.
class BackgroundMissionPool {
    std::vector<des::OrderPtr> m_missions;

public:
    void add(const des::OrderPtr& order) {
        m_missions.push_back(order);
        DES_LOG_DEBUG(rclcpp::get_logger("des.mission.background"), "Background mission added - list size: %zu", m_missions.size());
    }

    bool has() const {
        return !m_missions.empty();
    }

    std::size_t size() const {
        return m_missions.size();
    }

    void clear() {
        m_missions.clear();
    }

    // First-feasible background mission. Two acceptance paths:
    //   direct          — post-BG battery covers next-scheduled reserve.
    //   charge-fallback — dock window before next dispatch refills the deficit.
    //
    //   Budget calculation:
    //   - Time:
    //      - until start of next scheduled mission
    //      - until start of next scheduled mission minus charge time
    //  - Energy:
    //      - rest of battery
    //      - rest of battery minus energy used by next scheduled mission
    //
    //   On success the accepted mission is removed from the pool and returned.
    //   TODO: refactor
    des::OrderPtr acceptFeasible(
        const ISimContext& ctx,
        double safetyMarginWh = kBackgroundEnergySafetyMarginWh
    ) {
        if (m_missions.empty()) { return nullptr; }

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
        auto it = std::find_if(m_missions.begin(), m_missions.end(),
            [&](const des::OrderPtr& c) {
                result = evaluate(c);
                return result.has_value();
            });

        if (it == m_missions.end()) {
            DES_LOG_INFO(rclcpp::get_logger("des.mission.background"),
                        "No feasible background mission (battery=%.1fWh, reserved=%.1fWh, safety=%.1fWh)",
                        currentWh, reservedWh, safetyMarginWh);
            return nullptr;
        }

        const auto accepted = *it;
        DES_LOG_INFO(rclcpp::get_logger("des.mission.background"),
                    "Accept background %d (type=%s, est=%.1fWh, dur=%.0fs, reserve=%.1fWh, battery=%.1fWh, mode=%s)",
                    accepted->id, accepted->type.c_str(),
                    result->bgEnergy, result->bgDuration, reservedWh, currentWh, result->mode);
        m_missions.erase(it);
        return accepted;
    }
};
