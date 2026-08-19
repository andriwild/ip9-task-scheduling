#pragma once

#include "engine/contracts/i_sim_clock.h"
#include "engine/contracts/i_world_model.h"
#include "util/types.h"

namespace des {

struct SimConfig;

// Read-only slice of the simulation
struct EstimationView {
    const IWorldModel& world;
    const ISimClock& clock;
    const SimConfig& cfg;
};

}  // namespace des
