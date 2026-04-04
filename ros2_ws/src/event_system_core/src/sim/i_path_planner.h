#pragma once

#include <optional>
#include <string>

class IPathPlanner {
public:
    virtual ~IPathPlanner() = default;
    virtual std::optional<double> calcDistance(const std::string& from, const std::string& to, bool useCache) = 0;
};
