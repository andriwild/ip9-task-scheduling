#pragma once

#include "engine/contracts/i_behaviour_tree_access.h"
#include "engine/contracts/i_config_access.h"
#include "engine/contracts/i_event.h"
#include "engine/contracts/i_event_sink.h"
#include "engine/contracts/i_mission_board.h"
#include "engine/contracts/i_notifier.h"
#include "engine/contracts/i_path_planning.h"
#include "engine/contracts/i_person_registry.h"
#include "engine/contracts/i_rng_access.h"
#include "engine/contracts/i_robot_access.h"
#include "engine/contracts/i_sim_clock.h"
#include "engine/contracts/i_world_model.h"

namespace des {

class ISimContext
    : public ISimClock
    , public IEventSink
    , public IBehaviorTreeAccess
    , public IRobotAccess
    , public IPathPlanning
    , public INotifier
    , public IMissionBoard
    , public IPersonRegistry
    , public IWorldModel
    , public IConfigAccess
    , public IRngAccess {
public:
    ~ISimContext() override = default;
};

}  // namespace des
