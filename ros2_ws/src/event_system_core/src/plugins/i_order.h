#pragma once

#include <memory>
#include <string>
#include <optional>

#include "../util/types.h"


namespace des {

struct IOrder {
    int id           = 0;
    int dispatchTime = 0;
    std::string type;
    std::optional<int> deadline;
    std::string description;
    MissionState state = PENDING;

    virtual ~IOrder() = default;

};

using OrderPtr  = std::shared_ptr<IOrder>;
using OrderList = std::vector<OrderPtr>;

}
