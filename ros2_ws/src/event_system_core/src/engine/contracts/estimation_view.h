#pragma once

#include "engine/contracts/i_sim_clock.h"
#include "engine/contracts/i_world_model.h"
#include "util/types.h"

// Read-only slice of the simulation a cost estimate is allowed to see.
// Deliberately free of IEventSink, IRobotAccess and IMissionBoard: estimating
// must not move the robot, push events or touch the mission channels.
struct EstimationView {
    const IWorldModel& world;
    const ISimClock& clock;
    const des::SimConfig& cfg;
};
