#pragma once

#include <random>
#include <vector>

namespace rnd {
    inline double uni(std::mt19937& rng, const double from = 0.0, const double to = 1.0) {
        std::uniform_real_distribution dist(from, to);
        return dist(rng);
    }

    inline double normal(std::mt19937& rng, const double mean, const double std) {
        std::normal_distribution<double> dist(mean, std);
        return dist(rng);
    }

    inline double exponential(std::mt19937& rng, const double mean) {
        std::exponential_distribution<double> dist(1.0 / mean);
        return dist(rng);
    }

    inline double logNormal(std::mt19937& rng, const double mu, const double sigma) {
        std::lognormal_distribution<double> dist(mu, sigma);
        return dist(rng);
    }

    inline int discrete_dist(std::mt19937& rng, const std::vector<double>& list) {
        std::discrete_distribution<int> dist(list.begin(), list.end());
        return dist(rng);
    }

}
