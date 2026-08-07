#pragma once

#include "model/order.h"

namespace des {

struct InformationOrder : IOrder {
    mutable double sampledDuration = -1.0;
};

}  // namespace des
