#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "model/order.h"

namespace des {

enum class SearchAbortReason { NONE, OUTSIDE, IN_BUILDING_FINDABLE, IN_BUILDING_UNREACHABLE };

enum class AccompanyPhase { NONE, SEARCH, ACCOMPANY, CONVERSATE_FOUND, CONVERSATE_DROPOFF };

struct AccompanyOrder : IOrder {
    std::string personName;
    std::string roomName;
    std::vector<std::string> plannedSearch;
    std::vector<std::string> remainingSearch;
    size_t scanIndex = 0;
    AccompanyPhase phase = AccompanyPhase::NONE;
    SearchAbortReason abortReason = SearchAbortReason::NONE;
};

}  // namespace des
