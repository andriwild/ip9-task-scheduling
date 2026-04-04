// Service test: ros2 service call /set_des_state event_system_msgs/srv/SetSystemState
// "{new_state: 1}"

#pragma once

#include <qfont.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "event_system_msgs/srv/set_system_state.hpp"


using SystemState = event_system_msgs::srv::SetSystemState;

class ControllerNode : public rclcpp::Node {
public:
  std::atomic<int> currentState{ SystemState::Request::PAUSE };

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
    int old_state = currentState.load();
    currentState.store(request->command_id);

    switch (request->command_id) {
        case SystemState::Request::RUN:
            response->message = "Running";
            break;
        case SystemState::Request::PAUSE:
            response->message = "Paused";
            break;
        case SystemState::Request::RESET:
            response->message = "Reset";
            break;
        case SystemState::Request::STEP:
            response->message = "Step";
            break;
        case SystemState::Request::EXIT:
            response->message = "Exit";
            break;
        default:
            response->success = false;
            response->message = "failed";
            currentState.store(old_state);
    }
  }

  rclcpp::Service<event_system_msgs::srv::SetSystemState>::SharedPtr m_subscription;
};
