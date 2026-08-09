#include "states.h"


#include "../../util/log.h"
#include "engine/contracts/i_sim_context.h"
#include "../../model/robot.h"
#include "accompany_plugin.h"

namespace des {

void SearchState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG("des.plugin.accompany.state", "Enter Search");
    robot.setSpeed(robot.getDriveSpeed());
}

void AccompanyState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG("des.plugin.accompany.state", "Enter Accompany");
    robot.setSpeed(accompanyConfig().accompanySpeed);
}

void ConversateState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG("des.plugin.accompany.state", "Enter Conversate");
    robot.setSpeed(robot.getDriveSpeed());
}
double ConversateState::getEnergyConsumption(const Robot& /*robot*/, const SimConfig& cfg) const {
    return cfg.energyConsumptionBase;
}

}  // namespace des
