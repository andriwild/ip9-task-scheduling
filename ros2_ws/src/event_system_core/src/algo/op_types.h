/*
 * Plain data for a route planning problem.
 * A node is a place the robot can visit.
 * It has a reward and the time and energy it costs to work there.
 * The parameters hold the start and end node, the limits and the robot values needed to turn a distance into time and energy.
 *
 */

#pragma once

#include <string>
#include <vector>

using Mat = std::vector<std::vector<float>>;

struct OpNode {
    std::string name;
    float reward        = 0.0f;
    float serviceTime   = 0.0f;
    float serviceEnergy = 0.0f;
};

struct OpParams {
    int startNodeId       = 0;
    int endNodeId         = 0;
    float timeBudget      = 0.0f;
    float energyBudget    = 0.0f;
    float initialSoc      = 0.0f;
    float endSocMin       = 0.0f;
    float socThreshold    = 0.0f;
    float maxEnergy       = 0.0f;
    float chargeTimePerWh = 0.0f;
    float chargeTimePerWhTapered = 0.0f;
    float cvEnergy        = 0.0f;
    float driveSpeed      = 1.0f;
    float driveEnergy     = 0.0f;
};
