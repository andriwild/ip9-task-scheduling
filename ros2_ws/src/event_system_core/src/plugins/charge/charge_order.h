#pragma once

#include <string>
#include "model/order.h"

inline constexpr const char* kChargeOrderType = "charge";

struct ChargeOrder : des::IOrder {
    std::string dockLocation;
};
