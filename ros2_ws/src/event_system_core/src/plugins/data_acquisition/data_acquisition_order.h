#pragma once

#include <string>
#include "model/order.h"

struct DataAcquisitionOrder : des::IOrder {
    std::string roomName;
};
