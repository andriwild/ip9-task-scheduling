#pragma once

#include <optional>
#include <string>

namespace des {

class Scheduler;

struct Journey {
    double duration;
    double distance;
};

class IPathPlanning {
public:
    virtual ~IPathPlanning() = default;
    virtual Journey scheduleArrival(const std::string& target) const = 0;
    virtual const Scheduler& getScheduler() const = 0;

    // Path distance in meters between two waypoints, independent of the
    // distance source (matrix or Nav2). nullopt = unknown waypoint / no path.
    virtual std::optional<double> getDistance(const std::string& from, const std::string& to) const = 0;
};

}  // namespace des
