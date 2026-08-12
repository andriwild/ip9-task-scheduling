#pragma once

#include "model/order.h"

namespace des {

class IBackgroundOrders {
public:
    virtual ~IBackgroundOrders() = default;
    virtual void addBackgroundOrder(const OrderPtr orderPtr) = 0;
    virtual bool hasBackgroundOrder() const = 0;
    virtual OrderPtr acceptFeasibleBackgroundOrder() = 0;
};

}  // namespace des
