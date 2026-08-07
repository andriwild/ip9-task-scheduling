/*
 * Gives each room a score for the search route.
 * The score starts at the role prior and moves towards what the robot has really seen.
 * Many hits raise the score, many misses lower it.
 * Rooms the robot never visited keep the prior.
 *
 */

#pragma once

#include <string>
#include <vector>

#include "algo/op_types.h"
#include "algo/search/role_prior.h"
#include "model/room.h"
#include "model/sighting.h"

namespace des {

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
    const std::vector<std::string>& allRooms,
    const std::vector<RoomType>& roomTypes,
    const std::vector<std::string>& roles,
    bool useRolePrior,
    double priorWeight,
    float workplacePrior)
{
    std::vector<OpNode> nodes;
    for (std::size_t i = 0; i < allRooms.size(); ++i) {
        const std::string& room = allRooms[i];
        const SightingCounts c = sightings.counts(person, room);
        const RoomType type = i < roomTypes.size() ? roomTypes[i] : RoomType::OTHER;
        const double probability = roomProbability(c.hits, c.misses, occupancyPrior(office == room, type, roles, useRolePrior, workplacePrior), priorWeight);
        nodes.push_back({room, static_cast<float>(probability)});
    }
    return nodes;
}

// naive approach - if a person was seeing in a room, its probability is increased
inline std::vector<OpNode> frequencyReward(const SightingLog& sightings, const std::string& person, const std::vector<std::string>& allRooms) {
    std::vector<OpNode> nodes;
    for(const auto& room: allRooms) {
        const int hits = sightings.counts(person, room).hits;
        if(hits > 0) {
            nodes.push_back({room, static_cast<float>(hits)});
        }
    }
    return nodes;
}

}  // namespace des
