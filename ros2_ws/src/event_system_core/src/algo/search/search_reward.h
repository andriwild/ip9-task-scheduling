#pragma once

#include <string>
#include <vector>

#include "algo/op_types.h"
#include "model/sighting.h"

// calculate the probability of a room using beta-binomial model
inline float roomProbability(int hits, int misses, bool isWorkplace) {
    const double k     = 4.0;
    const double p0    = isWorkplace ? 0.6 : 0.05;
    const double alpha = k * p0;
    const double beta  = k * (1.0 - p0);
    const double y     = hits;
    const double n     = hits + misses;
    return (alpha + y) / (alpha + beta + n);
}

inline std::vector<OpNode> occupancyProbability(const SightingLog& sightings, const std::string& person, const std::string& office, const std::vector<std::string>& allRooms) {
    std::vector<OpNode> nodes;
    for(const auto& room: allRooms) {
        const SightingCounts c = sightings.counts(person, room);
        nodes.push_back({room, roomProbability(c.hits, c.misses, office == room)});
    }
    return nodes;

}

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
