#pragma once

#include "plugins/i_order.h"

class IBackgroundMissions {
public:
    virtual ~IBackgroundMissions() = default;
    virtual void addBackgroundMission(const des::OrderPtr orderPtr) = 0;
    virtual bool hasBackgroundMission() const = 0;
    virtual des::OrderPtr acceptFeasibleBackgroundMission() = 0;
};
