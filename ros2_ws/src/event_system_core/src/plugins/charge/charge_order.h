#pragma once

#include <string>
#include "model/order.h"

namespace des {

inline constexpr const char* kChargeOrderType = "charge";

struct ChargeOrder : IOrder {
    std::string dockLocation;
};

}  // namespace des
