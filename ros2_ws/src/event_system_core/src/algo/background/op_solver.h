/*
 * Collection of Orienteering Problem helper functions.
 *
 */
#pragma once

#include <algorithm>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include "../op.h"
#include "../op_solver_common.h"


namespace des::op {

// Finds the nearest dock to charge the robot.
inline int nearestDock(const OpInstance& op, const int from) {
    const auto& docks = op.docks();
    int best = docks.front();
    for (const int s : docks) {
        if (op.distance(from, s) < op.distance(from, best)) {
            best = s;
        }
    }
    return best;
}

// Optimizes a tour by swapping two edges.
// A swap is accepted if the tour gets shorter.
// Only the part between from and to is processed.
inline void twoOpt(const OpInstance& op, std::vector<int>& tour, const int from, const int to) {
    const auto& p   = op.params();
    // the two nodes just outside the part, the route has to keep reaching them
    const int left  = from == 0 ? p.startNodeId : tour[from - 1];
    const int right = to == static_cast<int>(tour.size()) ? p.endNodeId : tour[to];

    const int N = static_cast<int>(tour.size());
    if (N < 2) {
        return;
    }

    auto idAt = [&](int k) -> int {
        if (k == 0) { return left; }
        if (k == N + 1) { return right; }
        return tour[k - 1];
    };

    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = from; i <= to; ++i) {
            for (int j = i + 2; j <= to; ++j) {
                const int a = idAt(i);
                const int b = idAt(i + 1);
                const int c = idAt(j);
                const int d = idAt(j + 1);

                // cut the edges a-b and c-d and join a-c and b-d instead,
                const float oldEdges = op.distance(a, b) + op.distance(c, d);
                const float newEdges = op.distance(a, c) + op.distance(b, d);
                // only a real gain, a swap that changes nothing would repeat forever
                if (newEdges - oldEdges < -1e-6f) {
                    std::reverse(tour.begin() + i, tour.begin() + j);
                    improved = true;
                }
            }
        }
    }
}


struct Cand { float value; std::size_t idx; int dock; };

// Calculate the score of a candidate and check its feasibility with or without additional charge stop.
inline std::optional<Cand> scoreCandidate(
    const OpInstance& op,
    std::vector<int>& route,
    const int curId,
    const std::size_t candListIdx,
    const int candNode
    ) {
    route.push_back(candNode);
    const bool directlyReachable = op.simulateRoute(route).feasible;
    route.pop_back();

    if (directlyReachable) {
        return Cand{ greedyValue(op, curId, candNode), candListIdx, -1 };
    }

    // add a charge stop, check again if the candidate is feasible
    int bestDock   = -1;
    float bestCost = std::numeric_limits<float>::max();
    for (const int dock : op.docks()) {
        route.push_back(dock);
        route.push_back(candNode);
        const auto withCharge = op.simulateRoute(route);
        if (withCharge.feasible && withCharge.time < bestCost) {
            bestCost = withCharge.time;
            bestDock = dock;
        }
        route.pop_back();
        route.pop_back();
    }
    if (bestDock >= 0) {
        return Cand{ greedyValue(op, curId, candNode), candListIdx, bestDock };
    }
    return std::nullopt;
}

inline std::vector<Cand> restrictedCandidateList(const std::vector<Cand>& scored, const float alpha) {
    const auto byValue = [](const Cand& a, const Cand& b) { return a.value < b.value; };
    const float vmax = std::max_element(scored.begin(), scored.end(), byValue)->value;
    const float vmin = std::min_element(scored.begin(), scored.end(), byValue)->value;
    const float threshold = vmax - alpha * (vmax - vmin);

    std::vector<Cand> rcl;
    for (const auto& cand : scored) {
        if (cand.value >= threshold) {
            rcl.push_back(cand);
        }
    }
    return rcl;
}

// The route was built without checking the way back to the end node.
// Adds charge stops or drops tasks until the robot gets there.
inline void repairToEnd(const OpInstance& op, std::vector<int>& route) {
    while (!op.simulateRoute(route, true).feasible) {
        if (!op.docks().empty() && !route.empty() && !op.isDock(route.back())) {
            // the battery does not last to the end node, so charge on the way
            route.push_back(nearestDock(op, route.back()));
        } else {
            if (route.empty()) {
                // no task left to drop, the route stays infeasible
                break;
            }
            // charging did not help, so the last task has to go.
            // the route ends with the dock added above, that one goes with it
            route.pop_back();
            if (!op.docks().empty() && !route.empty()) {
                route.pop_back();
            }
        }
    }
}



// Create a solution by adding a candidate from the rcl
// with a given randomness of alpha to get out of local minima.
// alpha = 0: pure greedy; alpha = 1: random
inline std::vector<int> greedyRandomizedConstruction(const OpInstance& op, const float alpha, const int seed) {
    std::mt19937 gen(seed);
    std::vector<int> route;
    std::vector<int> candidates = taskCandidates(op);
    int curId = op.params().startNodeId;

    while (!candidates.empty()) {
        std::vector<Cand> scored;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (auto cand = scoreCandidate(op, route, curId, i, candidates[i])) {
                scored.push_back(*cand);
            }
        }
        if (scored.empty()) {
            break;
        }

        const std::vector<Cand> rcl = restrictedCandidateList(scored, alpha);
        std::uniform_int_distribution<std::size_t> pick(0, rcl.size() - 1);
        const Cand chosen = rcl[pick(gen)];

        if (chosen.dock >= 0) {
            route.push_back(chosen.dock);
        }
        curId = candidates[chosen.idx];
        route.push_back(curId);

        std::swap(candidates[chosen.idx], candidates.back());
        candidates.pop_back();
    }

    repairToEnd(op, route);
    return route;
}

// Orienteering Problem heuristics
// GRASP control loop
// Repeatably create a solution using greedyRandomizedConstruction to find the route with the best reward.
inline std::vector<int> grasp(const OpInstance& op, const int maxIterations, const float alpha, const int seed, const bool useTwoOpt = true) {
    std::vector<int> bestSolution;
    float bestReward = -1.0f;
    float bestDrive  = std::numeric_limits<float>::max();
    int bestDocks = std::numeric_limits<int>::max();

    for (int i = 0; i < maxIterations; ++i) {
        auto solution = greedyRandomizedConstruction(op, alpha, seed + i);

        const float reward = op.routeReward(solution);
        const float drive  = op.routeDriveDistance(solution);
        const int dockVisits = static_cast<int>(std::ranges::count_if(solution, [&](int x) { return op.isDock(x); }));

        if (reward > bestReward
            || (reward == bestReward && dockVisits < bestDocks)
            || (reward == bestReward && dockVisits == bestDocks && drive < bestDrive)
        ) {
            bestSolution = solution;
            bestReward   = reward;
            bestDrive    = drive;
            bestDocks = dockVisits;
        }
    }

    // try to shorten the route by applying 2-opt between charge stops
    if (useTwoOpt) {
        int legStart = 0;
        for (int i = 0; i < static_cast<int>(bestSolution.size()); ++i) {
            if (op.isDock(bestSolution[i])) {
                twoOpt(op, bestSolution, legStart, i);
                legStart = i + 1;
            }
        }
        twoOpt(op, bestSolution, legStart, static_cast<int>(bestSolution.size()));
    }

    return bestSolution;
}

}  // namespace des::op

