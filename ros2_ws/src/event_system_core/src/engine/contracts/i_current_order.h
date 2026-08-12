#pragma once

#include "model/order.h"
#include "util/types.h"

namespace des {

class ICurrentOrder {
public:
    virtual ~ICurrentOrder() = default;
    virtual void setOrderPtr(const OrderPtr& orderPtr) = 0;
    virtual OrderPtr getOrderPtr() const = 0;
    virtual void updateOrderState(const OrderState& newState) = 0;
    virtual void completeOrder(const OrderPtr& order) = 0;
};

}  // namespace des
