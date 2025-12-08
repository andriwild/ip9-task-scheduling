#pragma once

#include <random>

inline std::mt19937 rng(42); // Fixed seed for reproducibility

namespace rnd {
    inline double getRandom(const double from, const double to) {
        std::uniform_real_distribution<double> dist(from, to);
        return dist(rng);
    }

    inline double getNormalDist(const double from, const double to) {
        std::normal_distribution<double> dist(from, to);
        return dist(rng);
    }

    inline double uni() {
    	int r;
    	do {
    		r = rand();
    	} while (r == 0);
    	return double(r)/RAND_MAX;
    }

    inline double exp(double mean) {
    	return -mean*log(uni());
    }
}

