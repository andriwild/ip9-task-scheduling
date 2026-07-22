#pragma once

#include <algorithm>
#include <vector>

#include "op.h"

namespace op_solver {

inline float greedyValue(const OpInstance& op, const int curId, const int candIdx) {
    const auto& p = op.params();
    const float d = op.distance(curId, candIdx);
    const float driveLoad_e = d * p.driveEnergy / p.energyBudget;
    const float driveLoad_t = d / p.driveSpeed  / p.timeBudget;
    const float driveLoad   = std::max({ driveLoad_e, driveLoad_t, 1e-12f });
    return op.node(candIdx).reward / driveLoad;
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
