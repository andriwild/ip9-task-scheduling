// Service test: ros2 service call /set_des_state event_system_msgs/srv/SetSystemState
// "{new_state: 1}"

#pragma once

#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "../../init/config_loader.h"
#include "../../util/types.h"
#include "event_system_msgs/msg/system_config.hpp"
#include "event_system_msgs/srv/set_system_config.hpp"


class ConfigNode final : public rclcpp::Node {
public:
    explicit ConfigNode() : Node("des_config_node") {
        m_subscription = this->create_service<event_system_msgs::srv::SetSystemConfig>(
            "/set_des_config",
            std::bind(&ConfigNode::topicCallback, this, std::placeholders::_1, std::placeholders::_2));

        m_publisher = this->create_publisher<event_system_msgs::msg::SystemConfig>(
            "/system_config", rclcpp::QoS(1).transient_local());

        // Load initial config
        const auto loadedConfig = ConfigLoader::loadSimConfig();
        if (loadedConfig.has_value()) {
            RCLCPP_INFO(this->get_logger(), "Initial Simulation Config loaded!");
            m_currentConfig = std::make_shared<des::SimConfig>(loadedConfig.value());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load sim_config.json, using defaults.");
            m_currentConfig = std::make_shared<des::SimConfig>();
            m_currentConfig->robotSpeed = 1.0;
            m_currentConfig->robotAccompanySpeed = 1.0;
            m_currentConfig->appointmentsPath = "appointments.json";
        }
        publishConfig();
    }

    std::shared_ptr<des::SimConfig> getConfig() {
        std::lock_guard lock(m_configMutex);
        return m_currentConfig;
    }

    bool isConfigDirty() {
        std::lock_guard lock(m_configMutex);
        return m_dirtyConfig;
    }

    void clearDirty() {
        std::lock_guard lock(m_configMutex);
        m_dirtyConfig = false;
    }

private:
    void topicCallback(
        const std::shared_ptr<event_system_msgs::srv::SetSystemConfig::Request> &request,
        const std::shared_ptr<event_system_msgs::srv::SetSystemConfig::Response> &response
    ) { {
            auto config = des::SimConfig{
                request->robot_speed,
                request->robot_accompany_speed,
                request->drive_time_std,
                request->conversation_probability,
                request->conversation_duration_std,
                request->conversation_duration_mean,
                request->time_buffer,
                request->energy_consumption_drive,
                request->energy_consumption_base,
                request->battery_capacity,
                request->initial_battery_capacity,
                request->charging_rate,
                request->low_battery_threshold,
                request->full_battery_threshold,
                static_cast<double>(request->arrival_mean),
                static_cast<double>(request->arrival_std),
                static_cast<double>(request->departure_mean),
                static_cast<double>(request->departure_std),
                des::distributionTypeFromString(request->arrival_distribution),
                des::distributionTypeFromString(request->departure_distribution),
                request->dock_location,
                request->cache_enabled,
                request->appointments_path,
                request->appointment_duration
            };
            std::lock_guard lock(m_configMutex);
            m_currentConfig = std::make_shared<des::SimConfig>(config);
            m_dirtyConfig = true;
        }

        ConfigLoader::saveSimConfig(SIM_CONFIG_FILE, m_currentConfig);
        publishConfig();
        response->success = true;
        response->message = "successful";
    }

    void publishConfig() {
        auto msg = event_system_msgs::msg::SystemConfig();
        {
            std::lock_guard lock(m_configMutex);
            msg.drive_time_std             = m_currentConfig->driveTimeStd;
            msg.robot_speed                = m_currentConfig->robotSpeed;
            msg.robot_accompany_speed      = m_currentConfig->robotAccompanySpeed;
            msg.conversation_probability   = m_currentConfig->conversationProbability;
            msg.conversation_duration_std  = m_currentConfig->conversationDurationStd;
            msg.conversation_duration_mean = m_currentConfig->conversationDurationMean;
            msg.time_buffer                = m_currentConfig->timeBuffer;
            msg.energy_consumption_drive   = m_currentConfig->energyConsumptionDrive;
            msg.energy_consumption_base    = m_currentConfig->energyConsumptionBase;
            msg.battery_capacity           = m_currentConfig->batteryCapacity;
            msg.initial_battery_capacity   = m_currentConfig->initialBatteryCapacity;
            msg.charging_rate              = m_currentConfig->chargingRate;
            msg.low_battery_threshold      = m_currentConfig->lowBatteryThreshold;
            msg.full_battery_threshold     = m_currentConfig->fullBatteryThreshold;
            msg.arrival_mean               = m_currentConfig->arrivalMean;
            msg.arrival_std                = m_currentConfig->arrivalStd;
            msg.departure_mean             = m_currentConfig->departureMean;
            msg.departure_std              = m_currentConfig->departureStd;
            msg.arrival_distribution       = des::distributionTypeToString(m_currentConfig->arrivalDistribution);
            msg.departure_distribution     = des::distributionTypeToString(m_currentConfig->departureDistribution);
            msg.dock_location              = m_currentConfig->dockLocation;
            msg.cache_enabled              = m_currentConfig->cacheEnabled;
            msg.appointments_path          = m_currentConfig->appointmentsPath;
            msg.appointment_duration       = m_currentConfig->appointmentDuration;
        }
        m_publisher->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Simulation configuration published!");
    }

    rclcpp::Service<event_system_msgs::srv::SetSystemConfig>::SharedPtr m_subscription;
    rclcpp::Publisher<event_system_msgs::msg::SystemConfig>::SharedPtr m_publisher;
    std::mutex m_configMutex;
    bool m_dirtyConfig{};
    std::shared_ptr<des::SimConfig> m_currentConfig;
};
