#pragma once

#include <optional>
#include <string>
#include "model/order.h"

namespace des {

struct DataAcquisitionOrder : IOrder {
    std::string roomName;
    std::optional<double> acquisitionInterval;
};

}  // namespace des
