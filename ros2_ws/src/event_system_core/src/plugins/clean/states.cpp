#include "states.h"


#include "../../util/log.h"
#include "engine/contracts/i_sim_context.h"
#include "../../model/robot.h"

namespace des {

void CleanState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG("des.plugin.clean.state", "Enter Clean");
    robot.setSpeed(robot.getDriveSpeed());
}

// Sweeping the room is part of the cleaning service, its energy is already
// modelled by the service duration at base load, not by the travel rate.
double CleanState::getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const {
    if (robot.isServicing()) {
        return cfg.energyConsumptionBase;
    }
    return RobotState::getEnergyConsumption(robot, cfg);
}

}  // namespace des
