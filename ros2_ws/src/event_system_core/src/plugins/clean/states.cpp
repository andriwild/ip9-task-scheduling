#include "states.h"

#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "engine/contracts/i_sim_context.h"
#include "../../model/robot.h"

void CleanState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.clean.state"), "Enter Clean");
    robot.setSpeed(robot.getDriveSpeed());
}
