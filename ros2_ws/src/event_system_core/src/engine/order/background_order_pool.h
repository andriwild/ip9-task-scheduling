#pragma once

#include <algorithm>
#include <cassert>
#include <climits>
#include <cstddef>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "util/log.h"
#include "model/order.h"
#include "plugins/order_registry.h"
#include "model/battery.h"
#include "model/robot.h"
#include "engine/contracts/i_sim_context.h"
#include "algo/background/op_instance_builder.h"
#include "algo/background/op_solver.h"
#include "plugins/charge/charge_order.h"

namespace des {

constexpr double kBackgroundEnergySafetyMarginWh = 5.0;
constexpr double kReserveMarginPerMissionWh      = 1.5;
constexpr int kGraspIterations = 200;
constexpr float kGraspAlpha    = 0.3f;

struct MissionReserve {
    double requiredWh = 0.0;
    std::string endLoc;
    std::size_t missionCount = 0;
};

// Computes the required reserve from a given time point in the future.
// Accumulates the required energy of the schedules missions.
// Takes into account that can be charged between mission.
inline MissionReserve computeMissionReserve(
    const ISimContext& ctx,
    const double socThreshold,
    const double capacityWh,
    const std::string& dockLoc
) {
    const auto cfg          = ctx.getConfig();
    const double netChargeW = cfg->chargingRate - cfg->energyConsumptionBase;

    std::vector<OrderPtr> orders;
    if (cfg->energyReserveStrategy == EnergyReserveStrategy::HORIZON) {
        orders = ctx.peekScheduledOrdersUntil(ctx.getTime() + cfg->energyReserveHorizon);
    } else if (const auto next = ctx.peekNextScheduledOrder()) {
        orders.push_back(next);
    }

    if (orders.empty()) {
        return { socThreshold, dockLoc, 0 };
    }

    std::vector<double> energyWh(orders.size(), 0.0);
    std::vector<double> endTime(orders.size(), 0.0);
    std::vector<bool> hasDuration(orders.size(), false);
    std::string endLoc = dockLoc;

    for (std::size_t i = 0; i < orders.size(); ++i) {
        const auto& plugin = OrderRegistry::instance().get(orders[i]->type);
        const std::string startLoc = i == 0
            ? plugin.targetLocation(*orders[i]).value_or(dockLoc)
            : dockLoc;
        if (i == 0) {
            endLoc = startLoc;
        }
        energyWh[i] = plugin.estimateMissionEnergy(*orders[i], ctx, startLoc);

        const double durationSec = plugin.estimateMissionDuration(*orders[i], ctx, startLoc);
        hasDuration[i] = durationSec > 0.0;
        endTime[i]     = orders[i]->dispatchTime + durationSec;
    }

    double requiredWh = socThreshold;
    for (std::size_t k = orders.size(); k > 0; --k) {
        const std::size_t i = k - 1;
        double creditWh = 0.0;
        if (i + 1 < orders.size() && hasDuration[i] && netChargeW > 0.0) {
            const double gapSec = orders[i + 1]->dispatchTime - endTime[i];
            creditWh = std::max(0.0, gapSec) * netChargeW / 3600.0;
        }
        requiredWh = std::max(socThreshold + energyWh[i], requiredWh + energyWh[i] - creditWh);
    }

    return { std::min(capacityWh, requiredWh), endLoc, orders.size() };
}

// Pool of opportunistic background missions executed to fill idle time.
class BackgroundOrderPool {
    std::vector<OrderPtr> m_missions;
    std::queue<OrderPtr> m_pending;
    int m_chargeOrderSeq = 0;  // negative-id generator for synthesized charge stops

public:
    void add(const OrderPtr& order) {
        m_missions.push_back(order);
        DES_LOG_DEBUG("des.mission.background", "Background mission added - list size: %zu", m_missions.size());
    }

    bool has() const {
        return !m_missions.empty() || !m_pending.empty();
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
    OrderPtr popPlanned() {
        if (m_pending.empty()) { return nullptr; }
        const auto order = m_pending.front();
        m_pending.pop();
        std::erase(m_missions, order);
        return order;
    }

    void invalidatePlan() {
        while (!m_pending.empty()) {
            const auto order = m_pending.front();
            m_pending.pop();
            if (order && order->type != kChargeOrderType) {
                m_missions.push_back(order);
            }
        }
    }

    void plan(
        const ISimContext& ctx,
        double safetyMarginWh = kBackgroundEnergySafetyMarginWh
    ) {
        m_pending = {};
        if (m_missions.empty()) { return; }

        // remove duplicate missions from the pool
        // this happens, if a background mission is dispatched before its successor is processed (mission jam)
        std::set<std::pair<std::string, std::string>> seen;
        std::erase_if(m_missions, [&](const OrderPtr& o) {
            const auto target = OrderRegistry::instance().get(o->type).targetLocation(*o);
            if (!target) {
                return false;
            }
            return !seen.emplace(o->type, *target).second;
        });

        const auto robot            = ctx.getRobot();
        const auto batStats         = robot->batteryStats();
        const double voltage        = robot->batteryVoltage();
        const double currentWh      = batStats.soc * batStats.capacity * voltage;
        const double capacityWh     = batStats.capacity * voltage;
        const auto cfg              = ctx.getConfig();
        const double socThreshold   = capacityWh / 100 * cfg->lowBatteryThreshold;
        const std::string& startLoc = robot->getLocation();
        const std::string& dockLoc  = cfg->dockLocation;
        const int now               = ctx.getTime();
        const double netChargeW     = cfg->chargingRate - cfg->energyConsumptionBase;

        // tour ends where the next scheduled mission starts; reserve is computed
        // from there so the endSocMin check matches the actual handover point.
        const auto reserve = computeMissionReserve(ctx, socThreshold, capacityWh, dockLoc);
        const std::string& endLoc = reserve.endLoc;

        // energy budget [Wh]: spendable on background before hitting the next-mission reserve.
        const double blockMarginWh = safetyMarginWh
            + kReserveMarginPerMissionWh * static_cast<double>(reserve.missionCount > 0 ? reserve.missionCount - 1 : 0);
        const double requiredWh   = std::min(capacityWh, reserve.requiredWh + blockMarginWh);
        const double energyBudget = currentWh - requiredWh;

        // time budget [s]: until the next hard stop (next dispatch or sim end).
        const auto nextDispatch = ctx.getNextScheduledDispatchTime();
        const auto simEndTime   = ctx.getSimulationEndTime();
        assert((nextDispatch.has_value() || simEndTime.has_value()) && "No end time (Simlation | Dispatch)");
        const int timeBudget    = std::min(nextDispatch.value_or(INT_MAX), simEndTime.value_or(INT_MAX)) - now;

        if (timeBudget <= 0) { return; }

        // netChargeW <= 0: charging never pays off, price stations out of the tour
        const float chargeTimePerWh = netChargeW > 0.0 ? static_cast<float>(3600.0 / netChargeW) : kChargingPricedOut;
        const double taperedW = netChargeW * cfg->taperFraction;
        const float chargeTimePerWhTapered = taperedW > 0.0 ? static_cast<float>(3600.0 / taperedW) : kChargingPricedOut;
        const float cvEnergy = static_cast<float>(cfg->cvThreshold * capacityWh);

        const OpBudgets budgets {
            .timeBudget      = static_cast<float>(timeBudget),
            .energyBudget    = static_cast<float>(std::max(energyBudget, kMinEnergyBudgetWh)),
            .initialSoc      = static_cast<float>(currentWh),
            .endSocMin       = static_cast<float>(requiredWh),
            .socThreshold    = static_cast<float>(socThreshold),
            .maxEnergy       = static_cast<float>((cfg->chargeToFull ? cfg->fullBatteryThreshold / 100.0 : cfg->cvThreshold) * capacityWh),
            .chargeTimePerWh = chargeTimePerWh,
            .chargeTimePerWhTapered = chargeTimePerWhTapered,
            .cvEnergy        = cvEnergy,
        };

        // builds a problem instance containing parameters (constraints, budget) and a list of locations to visit 
        const auto problem = buildMissionInstance(EstimationView{ctx, ctx, *cfg}, ctx, m_missions, startLoc, endLoc, budgets);
        if (!problem) {
            DES_LOG_DEBUG("des.mission.background", "No plannable background missions (pool=%zu)", m_missions.size());
            return;
        }

        // index based route (tour)
        const int graspSeed = static_cast<int>(ctx.activeSeed() + GRASP_SEED_OFFSET);
        const auto route = op_solver::grasp(problem->instance, cfg->graspIterations,
                                            static_cast<float>(cfg->graspAlpha), graspSeed);

        DES_LOG_DEBUG("des.mission.background", "Route: %s", formatRoute(*problem, route, startLoc, endLoc).c_str());


        // generate a tour of orderPtr, which the robot can process
        int chargeStops = 0;
        for (const int idx : route) {
            if (const auto& order = problem->orderByNode[idx]) {
                m_pending.push(order);
                auto it = std::find(m_missions.begin(), m_missions.end(), order);
                if(it != m_missions.end()) {
                    m_missions.erase(it);
                }
            } else {
                // station node -> execute the planned charge as a background order
                auto charge         = std::make_shared<ChargeOrder>();
                charge->id          = -1 - m_chargeOrderSeq++;
                charge->type        = kChargeOrderType;
                charge->execution   = ExecutionMode::BACKGROUND;
                charge->description = "Charge";
                charge->dockLocation = dockLoc;
                m_pending.push(charge);
                ++chargeStops;
            }
        }

        DES_LOG_DEBUG("des.mission.background",
                    "Planned %zu/%zu background missions (%d charge stops, time=%ds, energy=%.1fWh, reserve=%.1fWh over %zu scheduled, strategy=%s)",
                    m_pending.size(), m_missions.size(), chargeStops, timeBudget, energyBudget, requiredWh, reserve.missionCount,
                    energyReserveStrategyToString(cfg->energyReserveStrategy).c_str());
    }
};

}  // namespace des
