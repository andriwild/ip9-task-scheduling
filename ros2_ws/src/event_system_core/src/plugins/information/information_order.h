#pragma once

#include "plugins/i_order.h"

struct InformationOrder : des::IOrder {
    mutable double sampledDuration = -1.0;
};
