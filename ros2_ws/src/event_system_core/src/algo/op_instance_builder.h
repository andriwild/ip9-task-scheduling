#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "op.h"
#include "../model/i_sim_context.h"
#include "../plugins/i_order.h"
#include "../plugins/order_registry.h"
#include "../util/log.h"

// OP instance in index-space plus the mapping back to sim missions.
struct OpProblem {
    OpInstance instance;
    std::vector<des::OrderPtr> orderByNode;  // node idx -> mission; nullptr for start/station
};

// Human-readable route, e.g. "Start (start) -> Office (clean) -> Dock (charge) -> Lobby (information) -> End (end)".
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

// Energy/time budgets in sim units; drive cost rates are derived from the config.
struct OpBudgets {
    float timeBudget;
    float energyBudget;
    float initialSoc;
    float endSocMin;
    float socThreshold;
    float maxEnergy;
    float chargeTimePerWh;
    float chargeTimePerWhTapered;
    float cvEnergy;
};

namespace op_build {

struct PlannedNode {
    OpNode op;
    des::OrderPtr order;  // nullptr for a structural anchor (start / dock / end)
};

inline PlannedNode anchorNode(const std::string& location) {
    return { OpNode{ location, 0.0f, 0.0f, 0.0f }, nullptr };
}

inline std::optional<PlannedNode> serviceNode(const ISimContext& ctx, const des::OrderPtr& order) {
    const auto& plugin = OrderRegistry::instance().get(order->type);
    const auto target  = plugin.targetLocation(*order);
    if (!target) {
        DES_LOG_WARN(rclcpp::get_logger("des.algo.op"), "Mission %d (type=%s) has no target location, excluded from plan", order->id, order->type.c_str());
        return std::nullopt;
    }
    return PlannedNode{
        OpNode{
            *target,
            static_cast<float>(plugin.estimateReward(*order, ctx)),
            static_cast<float>(plugin.estimateServiceDuration(*order, ctx)),
            static_cast<float>(plugin.estimateServiceEnergy(*order, ctx)),
        },
        order,
    };
}

// build instance specific distance matrix
inline std::optional<Mat> distanceMatrix(const ISimContext& ctx, const std::vector<PlannedNode>& planned) {
    const size_t n = planned.size();
    Mat mat(n, std::vector<float>(n, 0.0f));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            const auto d = ctx.getDistance(planned[i].op.name, planned[j].op.name);
            if (!d) {
                DES_LOG_ERROR(rclcpp::get_logger("des.algo.op"), "No distance %s -> %s", planned[i].op.name.c_str(), planned[j].op.name.c_str());
                return std::nullopt;
            }
            mat[i][j] = static_cast<float>(*d);
            mat[j][i] = mat[i][j];
        }
    }
    return mat;
}

}  // namespace op_build

inline std::optional<OpProblem> buildMissionInstance(
    const ISimContext& ctx,
    const std::vector<des::OrderPtr>& missions,
    const std::string& startLoc,
    const std::string& endLoc,
    const OpBudgets& budgets
) {
    const auto cfg          = ctx.getConfig();
    const std::string& dock = cfg->dockLocation;
    const int startNodeId   = 0;
    const int dockNodeId    = 1;
    int endNodeId           = dockNodeId;

    // planned contains all the nodes / locations which can be used for the tour
    std::vector<op_build::PlannedNode> planned = {
        op_build::anchorNode(startLoc),
        op_build::anchorNode(dock)
    };

    // if the tour is not ending at the dock, the endlocation must also be inserted
    if (endLoc != dock) {
        planned.push_back(op_build::anchorNode(endLoc));
        endNodeId = 2;
    }

    const size_t anchorCount = planned.size();

    for (const auto& order : missions) {
        if (auto node = op_build::serviceNode(ctx, order)) {
            planned.push_back(std::move(*node));
        }
    }

    if (planned.size() == anchorCount) {
        return std::nullopt;
    }

    auto mat = op_build::distanceMatrix(ctx, planned);
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

    return OpProblem{
        OpInstance(std::move(nodes), std::move(*mat), { dockNodeId }, params),
        std::move(orderByNode),
    };
}
