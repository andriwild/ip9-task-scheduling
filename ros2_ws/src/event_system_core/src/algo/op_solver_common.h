/*
 * Shared parts of the route solvers.
 * greedyValue() rates a next node by its reward divided by how much
 * of the time or energy limit the drive there uses up, whichever is tighter. 
 * taskCandidates() lists the nodes a solver may pick.
 * start, end and charging stations are left out.
 *
 */

#pragma once

#include <algorithm>
#include <vector>

#include "op.h"

namespace des {

// TODO: clean up namespaces

namespace op_solver {

inline float greedyValue(const OpInstance& op, const int curId, const int candIdx) {
    const auto& p    = op.params();
    const auto& node = op.node(candIdx);
    const float d    = op.distance(curId, candIdx);
    const float load_e = (d * p.driveEnergy + node.serviceEnergy) / p.energyBudget;
    const float load_t = (d / p.driveSpeed  + node.serviceTime)   / p.timeBudget;
    const float maxLoad = std::max({ load_e, load_t, 1e-12f });
    return p.costAware ? node.reward / maxLoad : node.reward;
}

namespace detail {

inline std::vector<int> taskCandidates(const OpInstance& op) {
    const auto& p = op.params();
    std::vector<int> candidates;
    candidates.reserve(op.nodeCount());
    for (std::size_t i = 0; i < op.nodeCount(); ++i) {
        const int idx = static_cast<int>(i);
        if (idx != p.startNodeId && idx != p.endNodeId && !op.isStation(idx)) {
            candidates.push_back(idx);
        }
    }
    return candidates;
}

}  // namespace detail

}  // namespace op_solver

}  // namespace des
