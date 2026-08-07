#pragma once

#include <optional>
#include <string>
#include "model/order.h"

namespace des {

struct CleanOrder : IOrder {
    std::string roomName;
    std::optional<double> cleaningInterval;
};

}  // namespace des
