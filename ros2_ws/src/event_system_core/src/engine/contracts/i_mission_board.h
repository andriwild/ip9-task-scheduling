#pragma once

#include "engine/contracts/i_background_missions.h"
#include "engine/contracts/i_current_order.h"
#include "engine/contracts/i_interrupts.h"
#include "engine/contracts/i_scheduled_missions.h"

namespace des {

class IMissionBoard
    : public ICurrentOrder
    , public IScheduledMissions
    , public IBackgroundMissions
    , public IInterrupts {
public:
    ~IMissionBoard() override = default;
};

}  // namespace des
