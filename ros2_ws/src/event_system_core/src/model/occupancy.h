#pragma once

#include <algorithm>
#include <random>

#include "person.h"
#include "../util/rnd.h"
#include "../util/types.h"

namespace des {

inline double sampleOccupancyTime(std::mt19937& rng, DistributionType type, double mean, double std) {
    switch (type) {
        case DistributionType::UNIFORM:     return rnd::uni(rng, mean - std, mean + std);
        case DistributionType::EXPONENTIAL: return rnd::exponential(rng, mean);
        case DistributionType::LOG_NORMAL:  return rnd::logNormal(rng, mean, std);
        default:                            return rnd::normal(rng, mean, std);
    }
}

inline void sampleOccupancy(const SimConfig& config, std::mt19937& rng, int dayBase, Person& p) {
    p.arrivalTime   = dayBase + static_cast<int>(sampleOccupancyTime(rng, config.arrivalDistribution, config.arrivalMean, config.arrivalStd));
    p.departureTime = dayBase + static_cast<int>(sampleOccupancyTime(rng, config.departureDistribution, config.departureMean, config.departureStd));
    p.lunchTime     = dayBase + static_cast<int>(sampleOccupancyTime(rng, config.lunchDistribution, config.lunchMean, config.lunchStd));
    p.lunchDuration = std::max(60.0, rnd::normal(rng, config.lunchDurationMean, config.lunchDurationStd));
    p.lunchPending  = true;
}

}  // namespace des
