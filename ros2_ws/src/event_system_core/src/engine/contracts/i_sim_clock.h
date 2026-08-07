#pragma once

#include <optional>

namespace des {

class ISimClock {
public:
    virtual ~ISimClock() = default;
    virtual int getTime() const = 0;
    virtual std::optional<int> getSimulationEndTime() const = 0;
};

}  // namespace des
