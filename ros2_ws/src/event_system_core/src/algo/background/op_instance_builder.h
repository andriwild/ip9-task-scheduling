#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "../op.h"
#include "../op_build.h"
#include "engine/contracts/estimation_view.h"
#include "engine/contracts/i_path_planning.h"
#include "model/order.h"
#include "../../plugins/order_registry.h"
#include "../../util/log.h"

namespace des {

struct OpProblem {
    OpInstance instance;
    std::vector<des::OrderPtr> orderByNode;
};

inline std::string formatRoute(const OpProblem& problem, const std::vector<int>& route, const std::string& startLoc, const std::string& endLoc) {
    std::string s = startLoc + " (start)";
    for (const int idx : route) {
        const auto& order = problem.orderByNode[idx];
        const std::string tag = order ? order->type : "charge";
        s += " -> " + problem.instance.node(idx).name + " (" + tag + ")";
    }
    s += " -> " + endLoc + " (end)";
    return s;
}

namespace op_build {

inline std::optional<PlannedNode> serviceNode(const EstimationView& view, const des::OrderPtr& order) {
    const auto& plugin = OrderRegistry::instance().get(order->type);
    const auto target  = plugin.targetLocation(*order);
    if (!target) {
        DES_LOG_WARN(rclcpp::get_logger("des.algo.op"), "Mission %d (type=%s) has no target location, excluded from plan", order->id, order->type.c_str());
        return std::nullopt;
    }
    return PlannedNode{
        OpNode{
            *target,
            static_cast<float>(plugin.estimateReward(*order, view)),
            static_cast<float>(plugin.estimateServiceDuration(*order, view)),
            static_cast<float>(plugin.estimateServiceEnergy(*order, view)),
        },
        order,
    };
}

}  // namespace op_build

inline std::optional<OpProblem> buildMissionInstance(
    const EstimationView& view,
    const IPathPlanning& paths,
    const std::vector<des::OrderPtr>& missions,
    const std::string& startLoc,
    const std::string& endLoc,
    const OpBudgets& budgets
) {
    const des::SimConfig& cfg = view.cfg;
    const std::string& dock   = cfg.dockLocation;
    const int startNodeId   = 0;
    const int dockNodeId    = 1;
    int endNodeId           = dockNodeId;

    std::vector<op_build::PlannedNode> planned = {
        op_build::anchorNode(startLoc),
        op_build::anchorNode(dock)
    };

    if (endLoc != dock) {
        planned.push_back(op_build::anchorNode(endLoc));
        endNodeId = 2;
    }

    const size_t anchorCount = planned.size();

    for (const auto& order : missions) {
        if (auto node = op_build::serviceNode(view, order)) {
            planned.push_back(std::move(*node));
        }
    }

    if (planned.size() == anchorCount) {
        return std::nullopt;
    }

    auto mat = op_build::distanceMatrix(paths, planned);
    if (!mat) {
        return std::nullopt;
    }

    std::vector<OpNode> nodes;
    std::vector<des::OrderPtr> orderByNode;
    nodes.reserve(planned.size());
    orderByNode.reserve(planned.size());

    for (auto& node : planned) {
        nodes.push_back(std::move(node.op));
        orderByNode.push_back(std::move(node.order));
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
    };

    return OpProblem{
        OpInstance(std::move(nodes), std::move(*mat), { dockNodeId }, params),
        std::move(orderByNode),
    };
}

}  // namespace des
