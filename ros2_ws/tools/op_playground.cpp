#include <cmath>
#include <iostream>
#include <vector>

#include "../src/event_system_core/src/algo/background/op_solver.h"
#include "op_svg.h"

int main() {
    std::vector<OpNode> nodes = {
        { "Start",  0.0f, 0.0f, 0.0f },
        { "RoomA",  4.0f, 0.0f, 0.0f },
        { "RoomB",  7.0f, 0.0f, 0.0f },
        { "RoomC",  3.0f, 0.0f, 0.0f },
        { "RoomD",  9.0f, 0.0f, 0.0f },
        { "RoomE",  5.0f, 0.0f, 0.0f },
        { "RoomF",  6.0f, 0.0f, 0.0f },
        { "RoomG",  2.0f, 0.0f, 0.0f },
        { "RoomH", 12.0f, 0.0f, 0.0f },
        { "RoomI",  8.0f, 0.0f, 0.0f },
        { "RoomJ",  5.0f, 0.0f, 0.0f },
        { "RoomK", 10.0f, 0.0f, 0.0f },
    };

    std::vector<Point> pos = {
        { 15.0f, 15.0f },
        {  5.0f,  5.0f },
        { 25.0f,  6.0f },
        { 28.0f, 15.0f },
        { 26.0f, 25.0f },
        { 15.0f, 28.0f },
        {  5.0f, 25.0f },
        {  3.0f, 15.0f },
        { 20.0f, 20.0f },
        { 10.0f, 10.0f },
        { 22.0f, 10.0f },
        { 18.0f,  4.0f },
    };

    const size_t n = nodes.size();
    Mat distances(n, std::vector<float>(n, 0.0f));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            distances[i][j] = std::hypot(pos[i].x - pos[j].x, pos[i].y - pos[j].y);
            distances[j][i] = distances[i][j];
        }
    }

    std::vector<int> stations = {};

    OpParams params;
    params.startNodeId  = 0;
    params.endNodeId    = 0;
    params.timeBudget   = 30.0f;
    params.driveSpeed   = 1.0f;
    params.driveEnergy  = 0.0f;
    params.energyBudget = 1e9f;
    params.initialSoc   = 1e9f;
    params.maxEnergy    = 1e9f;
    params.endSocMin    = 0.0f;
    params.socThreshold = 0.0f;

    const OpInstance op(nodes, distances, stations, params);

    const int iterations = 200;
    const float alpha    = 0.3f;
    const int seed       = 42;
    const std::vector<int> route = op_solver::grasp(op, iterations, alpha, seed);

    std::cout << "timeBudget = " << params.timeBudget << "\n";
    std::cout << "route: " << nodes[params.startNodeId].name;
    for (const int idx : route) {
        std::cout << " -> " << nodes[idx].name;
    }
    std::cout << " -> " << nodes[params.endNodeId].name << "\n";
    std::cout << "reward = " << op.routeReward(route) << "\n";
    std::cout << "time   = " << op.simulateRoute(route, true).time << "\n";

    writeSvg(nodes, pos, route, params, stations, "tools/op_route.svg");
    return 0;
}
