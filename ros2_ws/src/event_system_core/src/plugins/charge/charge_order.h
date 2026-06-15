#pragma once

#include <string>
#include "plugins/i_order.h"

inline constexpr const char* kChargeOrderType = "charge";

struct ChargeOrder : des::IOrder {
    std::string dockLocation;
};
