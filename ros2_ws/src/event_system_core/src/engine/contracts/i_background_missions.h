#pragma once

#include "model/order.h"

namespace des {

class IBackgroundMissions {
public:
    virtual ~IBackgroundMissions() = default;
    virtual void addBackgroundMission(const OrderPtr orderPtr) = 0;
    virtual bool hasBackgroundMission() const = 0;
    virtual OrderPtr acceptFeasibleBackgroundMission() = 0;
};

}  // namespace des
