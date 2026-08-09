#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include "model/order.h"

namespace des {

struct CleanOrder : IOrder {
    std::string roomName;
    std::optional<double> cleaningInterval;
    std::size_t sweepIndex = 0;
};

}  // namespace des
