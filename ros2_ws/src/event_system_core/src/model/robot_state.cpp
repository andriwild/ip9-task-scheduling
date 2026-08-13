#include "robot_state.h"


#include "../util/log.h"
#include "robot.h"

namespace des {

void RobotState::enter(Robot& robot) {
    m_result = Result::RUNNING;
    robot.beginStateInterval(getType(), getName());
}

void RobotState::exit(Robot& robot) {
    m_result = Result::SUCCESS;
    robot.endStateInterval();
}

double RobotState::getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const {
    return robot.isDriving()
        ? cfg.energyConsumptionDrive
        : cfg.energyConsumptionBase;
}

void IdleState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG("des.robot.state", "Enter Idle");
    robot.setSpeed(robot.getDriveSpeed());
}
double IdleState::getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const {
    if (robot.isDriving() && robot.getTargetLocation() == robot.getIdleLocation()) {
        return cfg.energyConsumptionDrive;
    }
    if (robot.getLocation() == robot.getIdleLocation()) {
        return 0.0;
    }
    return cfg.energyConsumptionBase;
}

void ConversationState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG("des.robot.state", "Enter Conversation");
    robot.setSpeed(robot.getDriveSpeed());
}

double ConversationState::getEnergyConsumption(const Robot& /*robot*/, const SimConfig& cfg) const {
    return cfg.energyConsumptionBase;
}

void ChargeState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG("des.robot.state", "Enter Charge");
}
double ChargeState::getEnergyConsumption(const Robot& robot, const SimConfig& cfg) const {
    auto energyConsumption = cfg.energyConsumptionBase;
    if (robot.isDriving()) {
        energyConsumption = cfg.energyConsumptionDrive;
    } else if (robot.getLocation() == robot.getIdleLocation()) {
        return robot.chargingConsumption(cfg.chargingRate, cfg.energyConsumptionBase);
    }
    return energyConsumption;
}

}  // namespace des
