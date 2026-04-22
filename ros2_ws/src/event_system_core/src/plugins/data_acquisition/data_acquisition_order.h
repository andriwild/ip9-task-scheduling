#pragma once

#include <string>
#include "plugins/i_order.h"

struct DataAcquisitionOrder : des::IOrder {
    std::string roomName;
};
