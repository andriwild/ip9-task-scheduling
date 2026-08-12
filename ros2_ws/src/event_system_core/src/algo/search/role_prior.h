/*
 * How likely each role is to be found in each room type.
 * These are fixed start values, used before the robot has seen the person anywhere.
 * If a person has several roles, the highest value wins.
 *
 */

#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "model/room.h"

namespace des {


struct RolePrior {
    const char* role;
    float workplace;
    float classroom;
    float meeting;
    float kitchen;
    float other;
    float toilet;
};

// TODO: explain in more detail, get rid of magic numbers
inline const std::vector<RolePrior>& rolePriors() {
    static const std::vector<RolePrior> table = {
        { "Student",            0.03f, 0.12f, 0.02f, 0.10f, 0.05f, 0.02f },
        { "Teacher",            0.04f, 0.09f, 0.10f, 0.09f, 0.05f, 0.02f },
        { "Research Assistant", 0.05f, 0.03f, 0.08f, 0.12f, 0.05f, 0.02f },
        { "Employee",           0.05f, 0.01f, 0.07f, 0.10f, 0.06f, 0.02f },
        { "Chef",               0.03f, 0.01f, 0.01f, 0.30f, 0.06f, 0.02f },
    };
    return table;
}

inline float priorFor(const RolePrior& p, const RoomType type) {
    switch (type) {
        case RoomType::WORKPLACE: return p.workplace;
        case RoomType::CLASSROOM: return p.classroom;
        case RoomType::MEETING:   return p.meeting;
        case RoomType::KITCHEN:   return p.kitchen;
        case RoomType::TOILET:    return p.toilet;
        case RoomType::OTHER:     return p.other;
    }
    return p.other;
}

inline float rolePrior(const std::vector<std::string>& roles, const RoomType type, const float fallback) {
    float best = -1.0f;
    for (const auto& role : roles) {
        const auto& table = rolePriors();
        const auto it = std::ranges::find_if(table, [&role](const RolePrior& p) { return role == p.role; });
        if (it != table.end()) {
            best = std::max(best, priorFor(*it, type));
        }
    }
    return best < 0.0f ? fallback : best;
}

}  // namespace des
