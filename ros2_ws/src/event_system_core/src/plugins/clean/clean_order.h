#pragma once

#include <optional>
#include <string>
#include "model/order.h"

struct CleanOrder : des::IOrder {
    std::string roomName;
    std::optional<double> cleaningInterval;
};
