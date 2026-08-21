#include <cmath>
#include <iostream>
#include <vector>

#include "algo/background/op_solver.h"
#include "op_svg.h"

using des::OpNode;
using des::OpParams;
using des::OpInstance;

int main() {
    std::vector<OpNode> nodes = {
        { "Start",        0.0f, 0.0f, 0.0f },
        { "Termin",       0.0f, 0.0f, 0.0f },
        { "Dock",         0.0f, 0.0f, 0.0f },
        { "Büro 5.2A01",  4.0f, 2.0f, 1.5f },
        { "Büro 5.2A12",  7.0f, 2.0f, 1.5f },
        { "Sitzung 5.2B", 9.0f, 3.0f, 2.5f },
        { "Küche",        3.0f, 1.5f, 1.0f },
        { "Hörsaal",     12.0f, 4.0f, 3.5f },
        { "Büro 5.2A34",  5.0f, 2.0f, 1.5f },
        { "Labor",        8.0f, 3.0f, 2.5f },
        { "Büro 5.2B07",  6.0f, 2.0f, 1.5f },
    };

    std::vector<Point> pos = {
        {  2.0f, 15.0f },
        { 30.0f, 16.0f },
        { 16.0f, 17.5f },
        {  4.0f,  7.0f },
        { 12.0f,  3.0f },
        { 26.0f,  5.0f },
        {  4.0f, 21.0f },
        { 21.0f, 26.0f },
        { 29.0f, 25.0f },
        { 11.0f, 11.0f },
        { 25.0f, 11.0f },
    };

    const std::size_t n = nodes.size();
    des::Mat distances(n, std::vector<float>(n, 0.0f));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            distances[i][j] = std::hypot(pos[i].x - pos[j].x, pos[i].y - pos[j].y);
            distances[j][i] = distances[i][j];
        }
    }

    const std::vector<int> stations = { 2 };

    OpParams params;
    params.startNodeId  = 0;
    params.endNodeId    = 1;
    params.timeBudget   = 160.0f;
    params.driveSpeed   = 1.0f;
    params.driveEnergy  = 0.5f;
    params.initialSoc   = 40.0f;
    params.maxEnergy    = 40.0f;
    params.endSocMin    = 10.0f;
    params.socThreshold = 6.0f;
    params.energyBudget = params.initialSoc - params.endSocMin;
    params.chargeTimePerWh        = 0.30f;
    params.chargeTimePerWhTapered = 0.60f;
    params.cvEnergy               = 32.0f;
    params.costAware              = true;
    params.openEnd                = false;

    const OpInstance op(nodes, distances, stations, params);
    const std::vector<int> route = des::op_solver::grasp(op, 200, 0.3f, 42);

    std::cout << "Zeitbudget   = " << params.timeBudget << "\n";
    std::cout << "Energiebudget= " << params.energyBudget << "\n";
    std::cout << "Route: " << nodes[params.startNodeId].name;
    for (const int idx : route) {
        std::cout << " -> " << nodes[idx].name;
    }
    std::cout << " -> " << nodes[params.endNodeId].name << "\n";
    const auto sim = op.simulateRoute(route, true);
    std::cout << "Reward = " << op.routeReward(route)
              << " | Zeit = " << sim.time
              << " | SoC am Ende = " << sim.socEnd
              << " | machbar = " << sim.feasible << "\n";

    writeSvg(nodes, pos, route, params, stations, "op_route.svg");
    return 0;
}
