/*
 * Builds the input for the search route planner.
 * Each room the robot search becomes a node. 
 * The cost to scan a room comes from its room tour. Start and end location are fixed.
 * Distances and the time and energy limits are added to each candidate.
 *
 */

#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "../op.h"
#include "../op_types.h"
#include "../op_build.h"
#include "engine/contracts/i_path_planning.h"
#include "engine/contracts/i_world_model.h"
#include "../../util/log.h"
#include "../../util/types.h"
#include "model/sim_config.h"
#include "model/room.h"


namespace des {

inline std::optional<OpInstance> buildSearchInstance(
    const IWorldModel& world,
    const IPathPlanning& paths,
    const SimConfig& cfg,
    const std::vector<OpNode>& roomNodes,
    const std::string& startLoc,
    const std::string& endLoc,
    const OpBudgets& budgets
) {
    constexpr int startNodeId = 0;
    std::vector planned = { op_build::anchorNode(startLoc) };

    // With a drop-off at the find location the route has no fixed end.
    // The appointment room is then an ordinary candidate like every other room.
    const bool openEnd = cfg.searchDropOffAtFind;

    int endNodeId = startNodeId;
    if (!openEnd && endLoc != startLoc) {
        endNodeId = static_cast<int>(planned.size());
        planned.push_back(op_build::anchorNode(endLoc));
    }
    const std::size_t anchorCount = planned.size();

    // precalculate time and energy usage of all candidates using sightseeing tour
    for (const auto& room : roomNodes) {
        if (room.name == startLoc || (!openEnd && room.name == endLoc)) {
            continue;
        }
        const RoomTour& tour = world.room(room.name).m_tour;
        if (tour.empty()) {
            DES_LOG_ERROR("des.algo.search", "No room tour for '%s'; excluded from search plan", room.name.c_str());
            continue;
        }
        const double scanTime    = tour.m_distance / cfg.robotSpeed;
        const double scanEnergy  = scanTime * cfg.energyConsumptionDrive / 3600.0;
        planned.push_back(op_build::PlannedNode{
            OpNode{ room.name, room.reward, static_cast<float>(scanTime), static_cast<float>(scanEnergy) },
            nullptr,
        });
    }

    if (planned.size() == anchorCount) {
        return std::nullopt;
    }

    // create submatrix containing only the candidates
    auto mat = op_build::distanceMatrix(paths, planned);
    if (!mat) {
        return std::nullopt;
    }

    std::vector<OpNode> nodes;
    nodes.reserve(planned.size());
    for (auto& [op, _] : planned) {
        nodes.push_back(std::move(op));
    }

    const auto driveEnergyPerMeter = static_cast<float>(cfg.energyConsumptionDrive / (3600.0 * cfg.robotSpeed));
    const OpParams params {
        .startNodeId     = startNodeId,
        .endNodeId       = endNodeId,
        .timeBudget      = budgets.timeBudget,
        .energyBudget    = budgets.energyBudget,
        .initialSoc      = budgets.initialSoc,
        .endSocMin       = budgets.endSocMin,
        .socThreshold    = budgets.socThreshold,
        .maxEnergy       = budgets.maxEnergy,
        .chargeTimePerWh = budgets.chargeTimePerWh,
        .chargeTimePerWhTapered = budgets.chargeTimePerWhTapered,
        .cvEnergy        = budgets.cvEnergy,
        .driveSpeed      = static_cast<float>(cfg.robotSpeed),
        .driveEnergy     = driveEnergyPerMeter,
        .costAware       = cfg.searchRouteStrategy == SearchRouteStrategy::COST_AWARE,
        .openEnd         = openEnd,
    };

    return OpInstance(std::move(nodes), std::move(*mat), {}, params);
}

}  // namespace des
