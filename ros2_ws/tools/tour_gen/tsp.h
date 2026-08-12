#pragma once

#include "VisLibRational.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cassert>
#include <cmath>
#include <random>
#include <utility>


namespace TSP {

struct Vec2 {
    double m_x = 0.0;
    double m_y = 0.0;
    VisLib::Point<VisLib::CT> m_exact;
};

struct Room {
    std::string m_name;
    std::vector<Vec2> m_footprint;
    Vec2 m_waypoint;
};

struct RoomTour {
    std::string m_roomName;
    Vec2 m_start;
    std::size_t m_steps = 0;
    double m_distance = 0.0;
    std::vector<Vec2> m_path;
    std::vector<std::vector<Vec2>> m_visPolys;
};

inline double distance(const Vec2& from, const Vec2& to) {
    return std::hypot(to.m_x - from.m_x, to.m_y - from.m_y);
}

// The only authority on how long a tour is: the points the robot actually drives.
inline double pathDistance(const RoomTour& tour) {
    double total = 0.0;
    for (std::size_t i = 1; i < tour.m_path.size(); ++i) {
        total += distance(tour.m_path[i - 1], tour.m_path[i]);
    }
    return total;
}



/**
* @brief Optimizes a TSP tour by swapping edges randomly
*
* Repeatedly removes two randomly edges and reverses the segment between them, reconnecting the tour.
* While iterating the temperature decreases and the chance to accept swapping
* edges which does not improve the tour distance decreases as well.
* Is the temperature high, the chance of accepting arbitrary swaps is greater.
* That enables to leave local minimums (difference to 2-opt).
*
* @param mat    distance matrix with values from each location to each other
* @param tour   the tour created by a construction algorithm
* @param T0     start temperature - the temperature goes down while iterating
* @param Tmin   the temperature to stop the iterations
* @param alpha  the factor to reduce T in each iteration
* @param trialsPerT  how often are 2 randomly edges swapped in each iteration (-1 = N x N) 
*
*/
// inline Tour simAnnealing(
//     const DistMat& mat,
//     const Tour& tour,
//     float T0 = 50.0f,
//     float Tmin = 0.01f,
//     float alpha = 0.995f,
//     int trialsPerT = -1
// ) {
//     Tour cur = tour;
//     Tour best = cur;
//     const size_t N = cur.order.size();
//     if (N < 4) { return cur; } // tour to short to improve
//     if (trialsPerT < 0) {
//         trialsPerT = static_cast<int>(N * N);
//     }

//     std::mt19937 gen(std::random_device{}());
//     std::uniform_real_distribution<float> uni(0.0f, 1.0f);
//     std::uniform_int_distribution<size_t> pickRnd(0, N - 1);

//     float T = T0;
//     while (T > Tmin) {
//         for (int t = 0; t < trialsPerT; ++t) {
//             size_t i = pickRnd(gen);
//             size_t j = pickRnd(gen);
//             if (i > j) { std::swap(i, j); }
//             if (j < i + 2) { continue; }  // adjacent or same edge
//             if (i == 0 && j == N - 1) { continue; } // wrap-around shares vertex 0

//             const int a = cur.order[i];
//             const int b = cur.order[i + 1];
//             const int c = cur.order[j];
//             const int d = cur.order[(j + 1) % N];

//             // calculate the delta of the tour distance
//             const float delta = (mat[a][c] + mat[b][d]) - (mat[a][b] + mat[c][d]);

//             // accept if the tour is shorter or by randomness (chance decreases over time)
//             if (delta < 0.0f || uni(gen) < std::exp(-delta / T)) {
//                 std::reverse(cur.order.begin() + i + 1, cur.order.begin() + j + 1);
//                 cur.distance += delta;
//                 if (cur.distance < best.distance) { best = cur; }
//             }
//         }
//         T *= alpha;
//     }
//     return best;
// }

// /*
// * @ brief 2-opt: scan all non-adjacent edge pairs, accept any improvement
// *
// * Repeatedly removes two edges and reverses the segment between them, reconnecting the tour.
// * This leads to prevent crossing edges and makes a tour distance shorter.
// *
// * @param mat    distance matrix with values from each location to each other
// * @param tour   the tour created by a construction algorithm
// *
// */
inline TSP::RoomTour twoOpt(TSP::RoomTour tour) {
    TSP::RoomTour out = tour;
    const size_t N = out.m_path.size();

    if (N < 4) { return out; }

    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 0; i < N - 1; ++i) {
            for (size_t j = i + 2; j < N; ++j) {
                if (i == 0 && j == N - 1) { continue; } // edges share vertex 0

                const TSP::Vec2& a = out.m_path[i];
                const TSP::Vec2& b = out.m_path[i + 1];
                const TSP::Vec2& c = out.m_path[j];
                const TSP::Vec2& d = out.m_path[(j + 1) % N];

                // calculate the delta of the tour distance
                const double delta = (distance(a,c) + distance(b,d)) - (distance(a,b) + distance(c,d));

                // accept every swap that makes the tour shorter
                if (delta < -1e-6) {
                    std::reverse(out.m_path.begin() + i + 1, out.m_path.begin() + j + 1);
                    out.m_distance += delta;
                    improved = true;
                }
            }
        }
    }
    return out;
}


/*
 * @brief A closed tour over location indices: order[0] is the start, the loop returns to order[0]
 *
 */
inline RoomTour nearestNeighbor(
    const RoomTour& tour,
    TSP::Room& room
) {
    if (tour.m_path.size() < 4) {
        return tour;
    }

    Vec2 start = tour.m_start;
    std::vector<Vec2> remaining;
    remaining.reserve(tour.m_path.size() - 2);
    // copy tour points without start at [0] and [size-1]
    for(size_t i = 1; i < tour.m_path.size() -1; ++ i) {
        remaining.push_back(tour.m_path[i]);
    }

    std::vector<Vec2> order;
    order.push_back(start);
    Vec2 currentPos = start;
    double tourDist = 0;

    while (!remaining.empty()) {
        double shortestPath = std::numeric_limits<double>::infinity();
        size_t bestSlot = 0;
        for (size_t j = 0; j < remaining.size(); ++j) {
            double dist = distance(currentPos, remaining[j]);
            if (dist < shortestPath) {
                shortestPath = dist;
                bestSlot = j;
            }
        }
        tourDist += shortestPath;
        currentPos = remaining[bestSlot];
        order.push_back(currentPos);
        std::swap(remaining[bestSlot], remaining.back());
        remaining.pop_back();
    }

    order.push_back(start);
    tourDist += distance(currentPos, start);

    RoomTour out = tour;
    out.m_path = order;
    out.m_distance = tourDist;
    out.m_steps = order.size();
    return out;
}
}
