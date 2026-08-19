// Service test: ros2 service call /set_des_state event_system_msgs/srv/SetSystemState
// "{new_state: 1}"

#pragma once

#include <qfont.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "event_system_msgs/srv/set_system_state.hpp"
#include "util/run_state.h"


namespace des {

using SystemState = event_system_msgs::srv::SetSystemState;

class ControllerNode : public rclcpp::Node {
public:
  std::atomic<RunState> currentState{ RunState::Pause };

  ControllerNode() : Node("des_controller_node") {
        m_subscription = this->create_service<SystemState>(
            "/set_des_state",
            std::bind(
                &ControllerNode::topicCallback, 
                this, 
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
  }

private:
  void topicCallback(
        const std::shared_ptr<SystemState::Request> &request,
        const std::shared_ptr<SystemState::Response> &response
    ) {
    response->success = true;
    switch (request->command_id) {
        case SystemState::Request::RUN:
            currentState.store(RunState::Run);
            response->message = "Running";
            break;
        case SystemState::Request::PAUSE:
            currentState.store(RunState::Pause);
            response->message = "Paused";
            break;
        case SystemState::Request::RESET:
            currentState.store(RunState::Reset);
            response->message = "Reset";
            break;
        default:
            response->success = false;
            response->message = "failed";
    }
  }

  rclcpp::Service<event_system_msgs::srv::SetSystemState>::SharedPtr m_subscription;
};

}  // namespace des
