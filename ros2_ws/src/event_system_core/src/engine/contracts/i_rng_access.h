#pragma once

#include <random>

namespace des {

class IRngAccess {
public:
    virtual ~IRngAccess() = default;
    virtual std::mt19937& worldRng() const = 0;
    virtual std::mt19937& robotRng() const = 0;
    virtual unsigned int activeSeed() const = 0;
};

}  // namespace des
