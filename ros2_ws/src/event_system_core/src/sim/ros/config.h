// Service test: ros2 service call /set_des_state event_system_msgs/srv/SetSystemState
// "{new_state: 1}"

#pragma once

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "../../init/config_loader.h"
#include "../../model/context.h"
#include "../../model/robot.h"
#include "../../util/types.h"
#include "event_system_msgs/msg/system_config.hpp"
#include "event_system_msgs/srv/set_system_config.hpp"

class ConfigNode : public rclcpp::Node {
public:
    ConfigNode() : Node("des_config_node") {
        m_subscription = this->create_service<event_system_msgs::srv::SetSystemConfig>(
            "/set_des_config",
            std::bind(&ConfigNode::topicCallback, this, std::placeholders::_1, std::placeholders::_2));

        m_publisher = this->create_publisher<event_system_msgs::msg::SystemConfig>(
            "/system_config", rclcpp::QoS(1).transient_local());

        // Load initial config
        auto loadedConfig = ConfigLoader::loadSimConfig("config/sim_config.json");
        if (loadedConfig.has_value()) {
            currentConfig = loadedConfig.value();
        } else {
            std::cerr << "Failed to load sim_config.json, using defaults." << std::endl;
        }
        publishConfig();
    }

    des::SimConfig getConfig() {
        std::lock_guard<std::mutex> lock(m_configMutex);
        return currentConfig;
    }

    void setRobot(std::shared_ptr<Robot> robot) { m_robot = robot; }
    void setContext(std::shared_ptr<SimulationContext> ctx) { m_ctx = ctx; }

private:
    des::SimConfig currentConfig{des::SimConfig{0.2, 0.3, 0.2, 0.1, 0.3, 0.4, 10, 30, "appointments.json"}};
    std::mutex m_configMutex;

    void topicCallback(
        const std::shared_ptr<event_system_msgs::srv::SetSystemConfig::Request> request,
        std::shared_ptr<event_system_msgs::srv::SetSystemConfig::Response> response) {
        auto config =
            des::SimConfig{
                request->find_person_probability,
                request->robot_speed,
                request->robot_accompany_speed,
                request->drive_std,
                request->conversation_found_std,
                request->conversation_drop_off_std,
                request->mission_overhead,
                request->time_buffer,
                request->appointments_path};
        {
            std::lock_guard<std::mutex> lock(m_configMutex);
            currentConfig = config;
        }
        ConfigLoader::saveSimConfig("config/sim_config.json", config);
        publishConfig();
        response->success = true;
        response->message = "successful";
        if (m_robot) {
            m_robot->setAccompanytSpeed(config.robotAccompanySpeed);
            m_robot->setSpeed(config.robotSpeed);
        }
        if (m_ctx) {
            m_ctx->setConfig(config);
        }
    }

    void publishConfig() {
        auto msg = event_system_msgs::msg::SystemConfig();
        {
            std::lock_guard<std::mutex> lock(m_configMutex);
            msg.find_person_probability = currentConfig.personFindProbability;
            msg.drive_std = currentConfig.driveStd;
            msg.robot_speed = currentConfig.robotSpeed;
            msg.robot_accompany_speed = currentConfig.robotAccompanySpeed;
            msg.conversation_found_std = currentConfig.conversationFoundStd;
            msg.conversation_drop_off_std = currentConfig.conversationDropOffStd;
            msg.mission_overhead = currentConfig.missionOverhead;
            msg.time_buffer = currentConfig.timeBuffer;
            msg.appointments_path = currentConfig.appointmentsPath;
        }
        m_publisher->publish(msg);
    }

    rclcpp::Service<event_system_msgs::srv::SetSystemConfig>::SharedPtr m_subscription;
    rclcpp::Publisher<event_system_msgs::msg::SystemConfig>::SharedPtr m_publisher;
    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<SimulationContext> m_ctx;
};
