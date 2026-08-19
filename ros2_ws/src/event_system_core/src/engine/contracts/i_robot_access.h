#pragma once

#include <memory>
#include <string>

#include "model/robot_state.h"
#include "util/types.h"
#include "util/point.h"

namespace des {

class Robot;

class IRobotAccess {
public:
    virtual ~IRobotAccess() = default;
    virtual Robot* getRobot() const = 0;
    virtual void changeRobotState(std::unique_ptr<RobotState> newState) const = 0;
    virtual void robotMoved(const std::string& location, double distance = 0) const = 0;
    virtual void robotMovedTo(const Point& position, double distance = 0.0) const = 0;
};

}  // namespace des
