#pragma once

#include <optional>
#include <string>
#include "model/order.h"

struct DataAcquisitionOrder : des::IOrder {
    std::string roomName;
    std::optional<double> acquisitionInterval;
};
