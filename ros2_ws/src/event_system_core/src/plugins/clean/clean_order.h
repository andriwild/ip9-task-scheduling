#pragma once

#include <string>
#include "plugins/i_order.h"

struct CleanOrder : des::IOrder {
    std::string roomName;
};
