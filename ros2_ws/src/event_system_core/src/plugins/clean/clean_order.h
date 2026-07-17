#pragma once

#include <optional>
#include <string>
#include "plugins/i_order.h"

struct CleanOrder : des::IOrder {
    std::string roomName;
    std::optional<double> cleaningInterval;
};
