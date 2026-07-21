#pragma once

#include <string>
#include <vector>

#include "plugins/i_order.h"

enum class SearchAbortReason { NONE, OUTSIDE, IN_BUILDING };

struct AccompanyOrder : des::IOrder {
    std::string personName;
    std::string roomName;
    std::vector<std::string> plannedSearch;
    SearchAbortReason abortReason = SearchAbortReason::NONE;
};
