#pragma once

#include "model/order.h"

class IInterrupts {
public:
    virtual ~IInterrupts() = default;
    virtual bool pushInterrupt(const des::OrderPtr& order) = 0;
    virtual void popInterrupt(const des::OrderPtr& completedOrder) = 0;
    virtual bool hasActiveInterrupt() const = 0;
};
