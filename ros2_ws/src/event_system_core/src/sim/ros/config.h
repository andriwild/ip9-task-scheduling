// Service test: ros2 service call /set_des_state event_system_msgs/srv/SetSystemState
// "{new_state: 1}"

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "../../util/types.h"

#include "event_system_msgs/srv/set_system_config.hpp"


class ConfigNode: public rclcpp::Node {
public:
    std::atomic<des::SimConfig> currentConfig { 
        des::SimConfig {
            0.2,
            0.3,
            0.2,
            0.1,
            0.3,
            0.4,
            10,
            30
        }
    };

    ConfigNode() : Node("des_config_node") {
        m_subscription = this->create_service<event_system_msgs::srv::SetSystemConfig>(
            "/set_des_config", 
            std::bind(&ConfigNode::topicCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

    bool isDirty() { return m_dirty; }
    void configUpdated() { m_dirty = false; }

private:
    void topicCallback(
        const std::shared_ptr<event_system_msgs::srv::SetSystemConfig::Request> request,
        std::shared_ptr<event_system_msgs::srv::SetSystemConfig::Response> response) {
        auto config = des::SimConfig {
            request->find_person_probability,
            request->robot_speed,
            request->robot_escort_speed,
            request->drive_std,
            request->conversation_found_std,
            request->conversation_drop_off_std,
            request->mission_overhead,
            request->time_buffer
        };
        currentConfig.store(config);
        response->success = true;
        response->message = "successful";
        m_dirty = true;
    }

    rclcpp::Service<event_system_msgs::srv::SetSystemConfig>::SharedPtr m_subscription;
    bool m_dirty;
};
