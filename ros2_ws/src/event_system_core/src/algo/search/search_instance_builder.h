#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "../op.h"
#include "../op_types.h"
#include "../op_build.h"
#include "../../model/i_sim_context.h"
#include "../../util/log.h"
#include "../../util/types.h"

inline std::optional<OpInstance> buildSearchInstance(
    const ISimContext& ctx,
    const std::vector<OpNode>& roomNodes,
    const std::string& startLoc,
    const std::string& endLoc,
    const OpBudgets& budgets
) {
    const auto cfg = ctx.getConfig();

    const int startNodeId = 0;
    std::vector<op_build::PlannedNode> planned = { op_build::anchorNode(startLoc) };

    int endNodeId = startNodeId;
    if (endLoc != startLoc) {
        endNodeId = static_cast<int>(planned.size());
        planned.push_back(op_build::anchorNode(endLoc));
    }
    const std::size_t anchorCount = planned.size();

    for (const auto& room : roomNodes) {
        if (room.name == startLoc || room.name == endLoc) {
            continue;
        }
        const des::RoomTour* tour = ctx.roomTour(room.name);
        if (tour == nullptr || tour->m_path.empty()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.algo.search"), "No room tour for '%s'; excluded from search plan", room.name.c_str());
            continue;
        }
        const double scanTime    = tour->m_distance / cfg->robotSpeed;
        const double scanEnergy  = scanTime * cfg->energyConsumptionDrive / 3600.0;
        planned.push_back(op_build::PlannedNode{
            OpNode{ room.name, room.reward, static_cast<float>(scanTime), static_cast<float>(scanEnergy) },
            nullptr,
        });
    }

    if (planned.size() == anchorCount) {
        return std::nullopt;
    }

    auto mat = op_build::distanceMatrix(ctx, planned);
    if (!mat) {
        return std::nullopt;
    }

    std::vector<OpNode> nodes;
    nodes.reserve(planned.size());
    for (auto& node : planned) {
        nodes.push_back(std::move(node.op));
    }

    const auto driveEnergyPerMeter = static_cast<float>(cfg->energyConsumptionDrive / (3600.0 * cfg->robotSpeed));
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
        .driveSpeed      = static_cast<float>(cfg->robotSpeed),
        .driveEnergy     = driveEnergyPerMeter,
    };

    return OpInstance(std::move(nodes), std::move(*mat), {}, params);
}
