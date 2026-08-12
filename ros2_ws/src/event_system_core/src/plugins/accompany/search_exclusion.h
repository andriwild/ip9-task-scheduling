#pragma once

#include <algorithm>
#include <string>

#include "engine/contracts/i_sim_context.h"
#include "model/room.h"

namespace des {

inline bool isSearchExcluded(const ISimContext& ctx, const std::string& room) {
    const auto config = ctx.getConfig();
    if (room == config->dockLocation) {
        return true;
    }
    const RoomType type = ctx.room(room).m_roomType;
    return std::ranges::find(config->searchExcludedRoomTypes, type) != config->searchExcludedRoomTypes.end();
}

}  // namespace des
