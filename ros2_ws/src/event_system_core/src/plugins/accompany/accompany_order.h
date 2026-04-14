#pragma once

#include <string>

#include "../i_order.h"

struct AccompanyOrder : des::IOrder {
    std::string personName;
    std::string roomName;
};
