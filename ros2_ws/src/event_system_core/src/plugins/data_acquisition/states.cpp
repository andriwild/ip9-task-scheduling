#include "states.h"

#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "../../model/i_sim_context.h"
#include "../../model/robot.h"

void AcquireState::enter(Robot& robot) {
    RobotState::enter(robot);
    DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.data_acquisition.state"), "Enter Acquire");
    robot.setSpeed(robot.getDriveSpeed());
}
