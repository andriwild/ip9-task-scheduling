#pragma once

#include <optional>

class ISimClock {
public:
    virtual ~ISimClock() = default;
    virtual int getTime() const = 0;
    virtual std::optional<int> getSimulationEndTime() const = 0;
};
