#pragma once

#include <string>

#include "plugins/i_order.h"

struct AccompanyOrder : des::IOrder {
    std::string personName;
    std::string roomName;
};
