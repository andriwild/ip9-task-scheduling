#pragma once

#include "model/order.h"

struct InformationOrder : des::IOrder {
    mutable double sampledDuration = -1.0;
};
