#pragma once

#include <algorithm>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include "../op.h"
#include "../op_solver_common.h"

namespace op_solver {

// TODO: naming Station(s), dock?
inline int nearestStation(const OpInstance& op, const int from) {
    const auto& stations = op.stations();
    int best = stations.front();
    for (const int s : stations) {
        if (op.distance(from, s) < op.distance(from, best)) {
            best = s;
        }
    }
    return best;
}

inline void twoOpt(const OpInstance& op, std::vector<int>& tour, const int from, const int to) {
    const auto& p   = op.params();
    const int left  = (from == 0)                           ? p.startNodeId : tour[from - 1];
    const int right = (to == static_cast<int>(tour.size())) ? p.endNodeId   : tour[to];

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

                const float oldEdges = op.distance(a, b) + op.distance(c, d);
                const float newEdges = op.distance(a, c) + op.distance(b, d);
                if (newEdges - oldEdges < -1e-6f) {
                    std::reverse(tour.begin() + i, tour.begin() + j);
                    improved = true;
                }
            }
        }
    }
}

namespace detail {

struct Cand { float value; std::size_t idx; int station; };

inline std::optional<Cand> scoreCandidate(const OpInstance& op, std::vector<int>& route,
                                          const int curId, const std::size_t candListIdx, const int candNode) {
    route.push_back(candNode);
    const bool directlyReachable = op.simulateRoute(route).feasible;
    route.pop_back();

    if (directlyReachable) {
        return Cand{ greedyValue(op, curId, candNode), candListIdx, -1 };
    }

    int bestStation = -1;
    float bestCost  = std::numeric_limits<float>::max();
    for (const int st : op.stations()) {
        route.push_back(st);
        route.push_back(candNode);
        const auto withCharge = op.simulateRoute(route);
        if (withCharge.feasible && withCharge.time < bestCost) {
            bestCost    = withCharge.time;
            bestStation = st;
        }
        route.pop_back();
        route.pop_back();
    }
    if (bestStation >= 0) {
        return Cand{ greedyValue(op, curId, candNode), candListIdx, bestStation };
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

inline void repairToEnd(const OpInstance& op, std::vector<int>& route) {
    while (!op.simulateRoute(route, true).feasible) {
        if (!op.stations().empty() && !route.empty() && !op.isStation(route.back())) {
            route.push_back(nearestStation(op, route.back()));
        } else {
            if (route.empty()) {
                break;
            }
            route.pop_back();
            if (!op.stations().empty() && !route.empty()) {
                route.pop_back();
            }
        }
    }
}

}  // namespace detail

inline std::vector<int> greedyRandomizedConstruction(const OpInstance& op, const float alpha, const int seed) {
    std::mt19937 gen(seed);
    std::vector<int> route;
    std::vector<int> candidates = detail::taskCandidates(op);
    int curId = op.params().startNodeId;

    while (!candidates.empty()) {
        std::vector<detail::Cand> scored;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (auto cand = detail::scoreCandidate(op, route, curId, i, candidates[i])) {
                scored.push_back(*cand);
            }
        }
        if (scored.empty()) {
            break;
        }

        const std::vector<detail::Cand> rcl = detail::restrictedCandidateList(scored, alpha);
        std::uniform_int_distribution<std::size_t> pick(0, rcl.size() - 1);
        const detail::Cand chosen = rcl[pick(gen)];

        if (chosen.station >= 0) {
            route.push_back(chosen.station);
        }
        curId = candidates[chosen.idx];
        route.push_back(curId);

        std::swap(candidates[chosen.idx], candidates.back());
        candidates.pop_back();
    }

    detail::repairToEnd(op, route);
    return route;
}

inline std::vector<int> grasp(const OpInstance& op, const int maxIterations, const float alpha, const int seed, const bool useTwoOpt = true) {
    std::vector<int> bestSolution;
    float bestReward = -1.0f;
    float bestDrive  = std::numeric_limits<float>::max();
    int bestStations = std::numeric_limits<int>::max();

    for (int i = 0; i < maxIterations; ++i) {
        auto solution = greedyRandomizedConstruction(op, alpha, seed + i);

        const float reward = op.routeReward(solution);
        const float drive  = op.routeDriveDistance(solution);
        const int stationVisits = static_cast<int>(std::ranges::count_if(solution, [&](int x) { return op.isStation(x); }));

        if (reward > bestReward
            || (reward == bestReward && stationVisits < bestStations)
            || (reward == bestReward && stationVisits == bestStations && drive < bestDrive)
        ) {
            bestSolution = solution;
            bestReward   = reward;
            bestDrive    = drive;
            bestStations = stationVisits;
        }
    }

    if (useTwoOpt) {
        int legStart = 0;
        for (int i = 0; i < static_cast<int>(bestSolution.size()); ++i) {
            if (op.isStation(bestSolution[i])) {
                twoOpt(op, bestSolution, legStart, i);
                legStart = i + 1;
            }
        }
        twoOpt(op, bestSolution, legStart, static_cast<int>(bestSolution.size()));
    }

    return bestSolution;
}

}  // namespace op_solver
