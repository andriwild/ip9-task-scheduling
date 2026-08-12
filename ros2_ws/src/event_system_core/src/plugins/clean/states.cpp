#include "states.h"


#include "../../util/log.h"
#include "engine/contracts/i_sim_context.h"
#include "../../model/robot.h"
#include "clean_plugin.h"

namespace des {

void CleanState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG("des.plugin.clean.state", "Enter Clean");
    robot.setSpeed(robot.getDriveSpeed());
}

// Sweeping runs the drive and the cleaning unit at the same time, so it draws
// the configured cleaning power rather than the base load. Must stay in sync
// with CleanPlugin::estimateServiceEnergy, otherwise the planner reserves an
// amount the simulation never books.
double CleanState::getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const {
    if (robot.isServicing()) {
        return cleanConfig().cleaningPower;
    }
    return RobotState::getEnergyConsumption(robot, cfg);
}

}  // namespace des
