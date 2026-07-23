#pragma once

#include <string>
#include <vector>
#include "algo/op_types.h"

enum class SightingKind { PRESENT, ABSENT };
struct Sighting {
    int time;
    std::string personName;
    std::string location;
    SightingKind kind;
};

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

inline std::vector<OpNode> occupancyProbability(const std::vector<Sighting>& sightings, const std::string& person, const std::string& office, const std::vector<std::string>& allRooms) {
    std::vector<OpNode> nodes;
    for(const auto& room: allRooms) {
        int hits = 0, misses = 0;
        for(const auto& s: sightings) {
            if(s.personName == person && s.location == room) {
                s.kind == SightingKind::PRESENT ? hits ++ : misses++;
            }
        }
        nodes.push_back({room, roomProbability(hits, misses, office == room)});
    }
    return nodes;

}

inline std::vector<OpNode> frequencyReward(const std::vector<Sighting>& sightings, const std::string& person, const std::vector<std::string>& allRooms) {
    std::vector<OpNode> nodes;
    for(const auto& room: allRooms) {
        int hits = 0;
        for(const auto& s: sightings) {
            if(s.personName == person && s.location == room && s.kind == SightingKind::PRESENT) {
                hits++;
            }
        }
        if(hits > 0) {
            nodes.push_back({room, static_cast<float>(hits)});
        }
    }
    return nodes;
}
