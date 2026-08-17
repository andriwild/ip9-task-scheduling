/*
 * Simple solver for the search route.
 * It always adds the room with the best value for its cost,
 * but only if the full route still fits into the time and energy limits.
 * It stops when no room fits any more.
 *
 */

#pragma once

#include <vector>

#include "../op.h"
#include "../op_solver_common.h"

namespace des::op_solver {

inline std::vector<int> greedySearchOrder(const OpInstance& op) {
    std::vector<int> route;
    std::vector<int> candidates = taskCandidates(op);
    int cur = op.params().startNodeId;
    while (true) {
        int bestPos = -1;
        float bestVal = -1.0f;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            const int c = candidates[i];
            route.push_back(c);
            const bool feasible = op.simulateRoute(route, true).feasible;
            route.pop_back();
            if (!feasible) {
                continue;
            }
            const float v = greedyValue(op, cur, c);
            if (v > bestVal) {
                bestVal = v;
                bestPos = static_cast<int>(i);
            }
        }
        if (bestPos < 0) {
            break;
        }
        cur = candidates[bestPos];
        route.push_back(cur);
        std::swap(candidates[bestPos], candidates.back());
        candidates.pop_back();
    }
    return route;
}

}