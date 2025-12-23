// Service test: ros2 service call /set_des_state event_system_msgs/srv/SetSystemState
// "{new_state: 1}"

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <bitset>
#include <cmath>
#include <iostream>

#include "event_system_msgs/srv/set_system_state.hpp"


class ControllerNode : public rclcpp::Node {
public:
    std::atomic<int> currentState{0};

    ControllerNode() : Node("des_controller_node") {
        m_subscription = this->create_service<event_system_msgs::srv::SetSystemState>(
            "/set_des_state", 
            std::bind(&ControllerNode::topicCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    void topicCallback(
        const std::shared_ptr<event_system_msgs::srv::SetSystemState::Request> request,
        std::shared_ptr<event_system_msgs::srv::SetSystemState::Response> response) {
        std::cout << "Bits: " << std::bitset<8>(request->command_id) << std::endl;
        std::cout << "[CONTROLLER] message received" << std::endl;

        response->success = true;
        int newState = event_system_msgs::srv::SetSystemState::Request::PAUSE;

        switch (request->command_id) {
            case event_system_msgs::srv::SetSystemState::Request::RUN:
                newState = event_system_msgs::srv::SetSystemState::Request::RUN;
                response->message = "Running";
                break;
            case event_system_msgs::srv::SetSystemState::Request::PAUSE:
                newState = event_system_msgs::srv::SetSystemState::Request::PAUSE;
                response->message = "Paused";
                break;
            case event_system_msgs::srv::SetSystemState::Request::RESET:
                newState = event_system_msgs::srv::SetSystemState::Request::RESET;
                response->message = "Reset";
                std::cout << "RESET" << std::endl;
                break;
            default:
                response->success = false;
                response->message = "failed";
        }
        currentState.store(newState);
    }

    rclcpp::Service<event_system_msgs::srv::SetSystemState>::SharedPtr m_subscription;
};
