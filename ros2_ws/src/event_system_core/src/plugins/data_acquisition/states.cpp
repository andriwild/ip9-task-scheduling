#include "states.h"


#include "../../util/log.h"
#include "engine/contracts/i_sim_context.h"
#include "../../model/robot.h"

namespace des {

void AcquireState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG("des.plugin.data_acquisition.state", "Enter Acquire");
    robot.setSpeed(robot.getDriveSpeed());
}

}  // namespace des
