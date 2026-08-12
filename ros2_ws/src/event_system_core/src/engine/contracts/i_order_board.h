#pragma once

#include "engine/contracts/i_background_orders.h"
#include "engine/contracts/i_current_order.h"
#include "engine/contracts/i_interrupts.h"
#include "engine/contracts/i_scheduled_orders.h"

namespace des {

class IOrderBoard
    : public ICurrentOrder
    , public IScheduledOrders
    , public IBackgroundOrders
    , public IInterrupts {
public:
    ~IOrderBoard() override = default;
};

}  // namespace des
