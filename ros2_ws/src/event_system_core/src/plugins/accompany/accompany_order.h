#pragma once

#include <cstddef>
#include <deque>
#include <set>
#include <string>
#include <vector>

#include "model/order.h"
#include "util/point.h"

namespace des {

enum class SearchAbortReason { NONE, OUTSIDE, IN_BUILDING_FINDABLE, IN_BUILDING_UNREACHABLE };

enum class AccompanyPhase { NONE, SEARCH, ACCOMPANY, CONVERSATE_FOUND, CONVERSATE_DROPOFF };

// One stop of the search route. Owned by the order, not by the room, so a
// mission may extend its own route without touching the shared room tour.
struct ScanStop {
    Point point;
    Polygon visibility;
    bool drive = true;
};

struct AccompanyOrder : IOrder {
    std::string personName;
    std::string roomName;
    std::vector<std::string> plannedSearch;
    std::vector<std::string> remainingSearch;
    std::string scanRoom;
    std::deque<ScanStop> scanQueue;
    std::set<std::string> identified;
    std::set<std::string> detoured;
    AccompanyPhase phase = AccompanyPhase::NONE;
    SearchAbortReason abortReason = SearchAbortReason::NONE;
};

}  // namespace des
