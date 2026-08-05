/*
 * Route planning problem: the nodes, the distances between them and the limits.
 * The main job is simulateRoute(): it drives the route step by step
 * and reports how much time it takes and how full the battery is at the end.
 * A route is feasible if it stays inside the time limit and never falls below the battery limit.
 * Charging stations refill the battery in the middle of a route.
 *
 */

#pragma once

#include <algorithm>
#include <vector>

#include "op_types.h"

class OpInstance {
    std::vector<OpNode> m_nodes;
    Mat m_mat;
    std::vector<int> m_stations;
    OpParams m_p;

public:
    OpInstance(std::vector<OpNode> nodes, Mat mat, std::vector<int> stations, const OpParams& params)
        : m_nodes(std::move(nodes)),
          m_mat(std::move(mat)),
          m_stations(std::move(stations)),
          m_p(params) {}

    const OpNode& node(const int idx) const { return m_nodes[idx]; }
    std::size_t nodeCount() const { return m_nodes.size(); }
    float distance(const int from, const int to) const { return m_mat[from][to]; }
    const std::vector<int>& stations() const { return m_stations; }
    const OpParams& params() const { return m_p; }
    float timeBudget() const { return m_p.timeBudget; }
    float energyBudget() const { return m_p.energyBudget; }

    bool isStation(const int idx) const {
        return std::ranges::find(m_stations, idx) != m_stations.end();
    }

    float routeReward(const std::vector<int>& route) const {
        float r = 0.0f;
        for (const int idx : route) {
            r += m_nodes[idx].reward;
        }
        return r;
    }

    float routeDriveDistance(const std::vector<int>& route) const {
        if (route.empty()) {
            return 0.0f;
        }
        float d = m_mat[m_p.startNodeId][route.front()];
        for (size_t i = 0; i + 1 < route.size(); ++i) {
            d += m_mat[route[i]][route[i + 1]];
        }
        d += m_mat[route.back()][m_p.endNodeId];
        return d;
    }

    float chargeDuration(const float from, const float to) const {
        float t = 0.0f;
        const float phaseOneEnd = std::min(to, m_p.cvEnergy);
        if (from < phaseOneEnd) {
            t += (phaseOneEnd - from) * m_p.chargeTimePerWh;
        }
        if (to > m_p.cvEnergy) {
            const float phaseTwoStart = std::max(from, m_p.cvEnergy);
            t += (to - phaseTwoStart) * m_p.chargeTimePerWhTapered;
        }
        return t;
    }

    struct Sim { bool feasible; float socEnd; float time; };

    Sim simulateRoute(const std::vector<int>& route, const bool toEnd = false) const {
        float time = 0.0f;
        float soc  = m_p.initialSoc;
        int prev   = m_p.startNodeId;

        for (const int idx : route) {
            const float d = m_mat[prev][idx];
            // time and energy to drive to the room
            soc  -= d * m_p.driveEnergy;
            time += d / m_p.driveSpeed;

            if (soc < 0.0f) {
                return { false, soc, time };
            }

            if (isStation(idx)) {
                time += chargeDuration(soc, m_p.maxEnergy);
                soc   = m_p.maxEnergy;
            } else {
                // time and energy for the work in the room (e.g. clean)
                soc  -= m_nodes[idx].serviceEnergy;
                time += m_nodes[idx].serviceTime;
                if (soc < 0.0f) {
                    return { false, soc, time };
                }
            }
            prev = idx;
        }

        if (toEnd) {
            const float d = m_mat[prev][m_p.endNodeId];
            soc  -= d * m_p.driveEnergy;
            time += d / m_p.driveSpeed;
        }

        const float minSoc = toEnd ? m_p.endSocMin : m_p.socThreshold;
        return { soc >= minSoc && time <= m_p.timeBudget, soc, time };
    }

    bool isFeasible(const std::vector<int>& route) const {
        return simulateRoute(route).feasible;
    }
};
