/*
 * Gives each room a score for the search route.
 * The score starts at the role prior and moves towards what the robot has really seen.
 * Many hits raise the score, many misses lower it.
 * Rooms the robot never visited keep the prior.
 *
 */

#pragma once

#include <ranges>
#include <string>
#include <vector>

#include "algo/op_types.h"
#include "algo/search/role_prior.h"
#include "model/room.h"
#include "model/sighting.h"

namespace des {

struct SearchRoom {
    std::string name;
    RoomType type = RoomType::OTHER;
};

constexpr float kUnseenRoomReward = 1e-3f;

// calculate the probability of a room using beta-binomial model
inline float roomProbability(int hits, int misses, float p0, double k) {
    const double alpha = k * p0;
    const double beta  = k * (1.0 - p0);
    const double y     = hits;
    const double n     = hits + misses;
    return (alpha + y) / (alpha + beta + n);
}

// get the prior for a room
// the workplace of a person has always the highest prior
// useRolePrior: if true the prior for rooms is depending on the person role
inline float occupancyPrior(bool isWorkplace, RoomType type, const std::vector<std::string>& roles, bool useRolePrior, float workplacePrior) {
    if (isWorkplace) {
        return workplacePrior;
    }
    if (!useRolePrior) {
        return 0.05f; // TODO: magic number
    }
    return rolePrior(roles, type, 0.05f);
}

// calculate the probability for each room according to its sightings
inline std::vector<OpNode> occupancyProbability(
    const SightingLog& sightings,
    const std::string& person,
    const std::string& office,
    const std::vector<SearchRoom>& allRooms,
    const std::vector<std::string>& roles,
    bool useRolePrior,
    double priorWeight,
    float workplacePrior)
{
    auto scored = allRooms | std::views::transform([&](const SearchRoom& room) {
        const SightingCounts c = sightings.counts(person, room.name);
        const float prior = occupancyPrior(office == room.name, room.type, roles, useRolePrior, workplacePrior);
        return OpNode{ room.name, roomProbability(c.hits, c.misses, prior, priorWeight) };
    });
    return { scored.begin(), scored.end() };
}

// naive approach - if a person was seeing in a room, its probability is increased
inline std::vector<OpNode> frequencyReward(const SightingLog& sightings, const std::string& person, const std::vector<SearchRoom>& allRooms) {
    auto scored = allRooms | std::views::transform([&](const SearchRoom& room) {
        const int hits = sightings.counts(person, room.name).hits;
        return OpNode{ room.name, hits > 0 ? static_cast<float>(hits) : kUnseenRoomReward };
    });
    return { scored.begin(), scored.end() };
}

}  // namespace des
