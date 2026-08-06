#pragma once

#include "engine/contracts/i_sim_clock.h"
#include "engine/contracts/i_world_model.h"
#include "util/types.h"

// Read-only slice of the simulation
struct EstimationView {
    const IWorldModel& world;
    const ISimClock& clock;
    const des::SimConfig& cfg;
};
