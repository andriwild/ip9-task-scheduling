#pragma once

#include "model/order.h"

namespace des {

class IInterrupts {
public:
    virtual ~IInterrupts() = default;
    virtual bool pushInterrupt(const OrderPtr& order) = 0;
    virtual void popInterrupt(const OrderPtr& completedOrder) = 0;
    virtual bool hasActiveInterrupt() const = 0;
};

}  // namespace des
