#pragma once

#include <random>

class IRngAccess {
public:
    virtual ~IRngAccess() = default;
    virtual std::mt19937& worldRng() const = 0;
    virtual std::mt19937& robotRng() const = 0;
};
