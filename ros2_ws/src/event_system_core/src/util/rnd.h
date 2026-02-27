#pragma once

#include <random>

inline std::mt19937 rng(42);

namespace rnd {
    inline double uni(const double from = 0.0, const double to = 1.0) {
        std::uniform_real_distribution dist(from, to);
        return dist(rng);
    }

    inline double getNormalDist(const double mean, const double std) {
        std::normal_distribution<double> dist(mean, std);
        return dist(rng);
    }
}