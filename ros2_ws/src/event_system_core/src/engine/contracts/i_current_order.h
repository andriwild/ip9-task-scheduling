#pragma once

#include "model/order.h"
#include "util/types.h"

namespace des {

class ICurrentOrder {
public:
    virtual ~ICurrentOrder() = default;
    virtual void setOrderPtr(const OrderPtr& orderPtr) = 0;
    virtual OrderPtr getOrderPtr() const = 0;
    virtual void updateOrderState(const MissionState& newState) = 0;
    virtual void completeOrder(const OrderPtr& appt) = 0;
};

}  // namespace des
