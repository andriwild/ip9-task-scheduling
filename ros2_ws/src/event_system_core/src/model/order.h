/*
 * An order represents a mission with an execution type.
 * Depending on its type, the order has a dispatch time.
 */
#pragma once

#include <memory>
#include <string>
#include <optional>

#include "util/types.h"


namespace des {

struct IOrder {
    int id           = 0;
    int dispatchTime = 0;
    std::string type;
    std::optional<int> deadline;
    std::string description;
    OrderState state = PENDING;
    ExecutionMode execution = ExecutionMode::BACKGROUND;

    virtual ~IOrder() = default;
};

using OrderPtr  = std::shared_ptr<IOrder>;
using OrderList = std::vector<OrderPtr>;

}
