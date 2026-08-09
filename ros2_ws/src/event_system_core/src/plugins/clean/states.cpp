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

}  // namespace des
