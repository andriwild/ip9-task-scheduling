#pragma once

#include <algorithm>
#include <cassert>
#include <climits>
#include <cstddef>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "../../plugins/i_order.h"
#include "../../plugins/order_registry.h"
#include "../battery.hpp"
#include "../robot.h"
#include "../i_sim_context.h"
#include "../../algo/op_instance_builder.h"

constexpr double kBackgroundEnergySafetyMarginWh = 5.0;
constexpr int kGraspIterations = 200;
constexpr float kGraspAlpha    = 0.3f;
constexpr int kGraspSeed       = 42;

// Pool of opportunistic background missions executed to fill idle time.
class BackgroundMissionPool {
    std::vector<des::OrderPtr> m_missions;
    std::queue<des::OrderPtr> m_pending;

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
        m_pending = {};
    }

    bool hasPlanned() const {
        return !m_pending.empty();
    }

    // Next mission of the current plan; also removed from the pool.
    des::OrderPtr popPlanned() {
        if (m_pending.empty()) { return nullptr; }
        const auto order = m_pending.front();
        m_pending.pop();
        std::erase(m_missions, order);
        return order;
    }

    void plan(
        const ISimContext& ctx,
        double safetyMarginWh = kBackgroundEnergySafetyMarginWh
    ) {
        m_pending = {};
        if (m_missions.empty()) { return; }

        const auto robot            = ctx.getRobot();
        const auto batStats         = robot->m_bat->getStats();
        const double currentWh      = batStats.soc * batStats.capacity * Battery::kVoltage;
        const double capacityWh     = batStats.capacity * Battery::kVoltage;
        const auto cfg              = ctx.getConfig();
        const std::string& startLoc = robot->getLocation();
        const std::string& dockLoc  = cfg->dockLocation;
        const int now               = ctx.getTime();
        const double netChargeW     = cfg->chargingRate - cfg->energyConsumptionBase;

        // tour ends where the next scheduled mission starts; reserve is computed
        // from there so the endSocMin check matches the actual handover point.
        std::string endLoc = dockLoc;
        double reservedWh  = 0.0;
        if (auto next = ctx.peekNextScheduledOrder()) {
            const auto& nextPlugin = OrderRegistry::instance().get(next->type);
            endLoc     = nextPlugin.targetLocation(*next).value_or(dockLoc);
            reservedWh = nextPlugin.estimateMissionEnergy(*next, ctx, endLoc);
        }

        // energy budget [Wh]: spendable on background before hitting the next-mission reserve.
        // requiredWh = energy the next scheduled mission needs + safety floor.
        const double requiredWh   = reservedWh + safetyMarginWh;
        const double energyBudget = currentWh - requiredWh;

        // time budget [s]: until the next hard stop (next dispatch or sim end).
        const auto nextDispatch = ctx.getNextScheduledDispatchTime();
        const auto simEndTime   = ctx.getSimulationEndTime();
        assert((nextDispatch.has_value() || simEndTime.has_value()) && "No end time (Simlation | Dispatch)");
        const int timeBudget    = std::min(nextDispatch.value_or(INT_MAX), simEndTime.value_or(INT_MAX)) - now;

        if (timeBudget <= 0) { return; }

        // netChargeW <= 0: charging never pays off, price stations out of the tour
        const float chargeTimePerWh = netChargeW > 0.0 ? static_cast<float>(3600.0 / netChargeW) : 1e9f;

        const OpBudgets budgets {
            .timeBudget      = static_cast<float>(timeBudget),
            .energyBudget    = static_cast<float>(std::max(energyBudget, 1e-3)),
            .initialSoc      = static_cast<float>(currentWh),
            .endSocMin       = static_cast<float>(requiredWh),
            .maxEnergy       = static_cast<float>(cfg->fullBatteryThreshold / 100.0 * capacityWh),
            .chargeTimePerWh = chargeTimePerWh,
        };

        const auto problem = buildOpInstance(ctx, m_missions, startLoc, endLoc, budgets);
        if (!problem) {
            DES_LOG_INFO(rclcpp::get_logger("des.mission.background"), "No plannable background missions (pool=%zu)", m_missions.size());
            return;
        }

        const auto route = problem->instance.grasp(kGraspIterations, kGraspAlpha, kGraspSeed);

        int chargeStops = 0;
        for (const int idx : route) {
            if (const auto& order = problem->orderByNode[idx]) {
                m_pending.push(order);
                auto it = std::find(m_missions.begin(), m_missions.end(), order);
                if(it != m_missions.end()) {
                    m_missions.erase(it);
                }
            } else {
                ++chargeStops;
            }
        }

        DES_LOG_INFO(rclcpp::get_logger("des.mission.background"),
                    "Planned %zu/%zu background missions (%d charge stops, time=%ds, energy=%.1fWh, reserve=%.1fWh)",
                    m_pending.size(), m_missions.size(), chargeStops, timeBudget, energyBudget, requiredWh);
    }
};
