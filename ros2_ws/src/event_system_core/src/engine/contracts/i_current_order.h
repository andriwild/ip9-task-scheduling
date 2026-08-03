#pragma once

#include "plugins/i_order.h"
#include "util/types.h"

class ICurrentOrder {
public:
    virtual ~ICurrentOrder() = default;
    virtual void setOrderPtr(const des::OrderPtr& orderPtr) = 0;
    virtual des::OrderPtr getOrderPtr() const = 0;
    virtual void updateOrderState(const des::MissionState& newState) = 0;
    virtual void completeOrder(const des::OrderPtr& appt) = 0;
};
