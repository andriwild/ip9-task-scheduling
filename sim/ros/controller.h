// Service test: ros2 service call /set_des_state des_msgs/srv/SetDesState "{new_state: 1}"

#pragma once

#include "des_msgs/srv/set_des_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include <bitset>
#include <cmath>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

enum SimState { IDLE = 0, RUN = 1 << 0, PAUSE = 1 << 1, RESET = 1 << 2 };

class ControllerNode : public rclcpp::Node {
public:
    std::atomic<SimState> currentState{IDLE};

    ControllerNode() : Node("des_controller_node") {

        subscription_ = this->create_service<des_msgs::srv::SetDesState>(
            "/set_des_state",
            std::bind(&ControllerNode::topicCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void topicCallback(
        const std::shared_ptr<des_msgs::srv::SetDesState::Request> request,
        std::shared_ptr<des_msgs::srv::SetDesState::Response> response
    ) {
        std::cout << "Bits: " << std::bitset<8>(request->new_state) << std::endl;
        std::cout << "[CONTROLLER] message received" << std::endl;

        response->success = true;
        switch (request->new_state) {
            case RUN:
                currentState.store(RUN);
                response->message = "Running";
                std::cout << "START" << std::endl;
                break;
            case PAUSE:
                currentState.store(PAUSE);
                response->message = "Paused";
                std::cout << "PAUSE" << std::endl;
                break;
            case IDLE:
                currentState.store(IDLE);
                response->message = "Idle";
                std::cout << "IDLE" << std::endl;
                break;
            case RESET:
                response->message = "Reset";
                currentState.store(RESET);
                std::cout << "RESET" << std::endl;
                break;
            default:
                response->success = false;
                response->message = "failed";
        }
    }

    rclcpp::Service<des_msgs::srv::SetDesState>::SharedPtr subscription_;
};
