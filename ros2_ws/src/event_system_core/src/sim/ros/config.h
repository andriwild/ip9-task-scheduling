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
#include "model/sim_config.h"


namespace des {

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
            m_currentConfig = std::make_shared<SimConfig>(loadedConfig.value());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load sim_config.json, using defaults.");
            m_currentConfig = std::make_shared<SimConfig>();
            m_currentConfig->robotSpeed = 1.0;
            m_currentConfig->scenarioPath = "appointments.json";
        }
        publishConfig();
    }

    std::shared_ptr<SimConfig> getConfig() {
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
    ) {
        {
            std::lock_guard lock(m_configMutex);
            auto config = *m_currentConfig;

            config.robotSpeed             = request->robot_speed;
            config.driveDelayMedian       = request->drive_delay_median;
            config.driveDelaySigma        = request->drive_delay_sigma;
            config.timeBuffer             = request->time_buffer;
            config.energyConsumptionDrive = request->energy_consumption_drive;
            config.energyConsumptionBase  = request->energy_consumption_base;
            config.batteryCapacity        = request->battery_capacity;
            config.initialBatteryCapacity = request->initial_battery_capacity;
            config.chargingRate           = request->charging_rate;
            config.lowBatteryThreshold    = request->low_battery_threshold;
            config.fullBatteryThreshold   = request->full_battery_threshold;
            config.arrivalMean            = request->arrival_mean;
            config.arrivalStd             = request->arrival_std;
            config.departureMean          = request->departure_mean;
            config.departureStd           = request->departure_std;
            config.arrivalDistribution    = distributionTypeFromString(request->arrival_distribution);
            config.departureDistribution  = distributionTypeFromString(request->departure_distribution);
            config.dockLocation           = request->dock_location;
            config.cacheEnabled           = request->cache_enabled;
            config.scenarioPath           = request->scenario_path;
            config.peopleSpawnLocation    = request->people_spawn_location;
            config.personIdentificationRange = request->person_detection_range;
            config.simStartTime           = request->sim_start_time;
            config.simDuration            = request->sim_duration;
            config.batteryVoltage         = request->battery_voltage;
            config.cvThreshold            = request->cv_threshold;
            config.taperFraction          = request->taper_fraction;
            config.chargeToFull           = request->charge_to_full;
            config.alwaysChargeAtDock     = request->always_charge_at_dock;

            m_currentConfig = std::make_shared<SimConfig>(config);
            m_dirtyConfig = true;
        }

        ConfigLoader::saveSimConfig(ConfigLoader::baseConfigPath(), m_currentConfig);
        publishConfig();
        response->success = true;
        response->message = "successful";
    }

    void publishConfig() {
        auto msg = event_system_msgs::msg::SystemConfig();
        {
            std::lock_guard lock(m_configMutex);
            msg.drive_delay_median         = m_currentConfig->driveDelayMedian;
            msg.drive_delay_sigma          = m_currentConfig->driveDelaySigma;
            msg.robot_speed                = m_currentConfig->robotSpeed;
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
            msg.arrival_distribution       = distributionTypeToString(m_currentConfig->arrivalDistribution);
            msg.departure_distribution     = distributionTypeToString(m_currentConfig->departureDistribution);
            msg.dock_location              = m_currentConfig->dockLocation;
            msg.cache_enabled              = m_currentConfig->cacheEnabled;
            msg.scenario_path              = m_currentConfig->scenarioPath;
            msg.people_spawn_location      = m_currentConfig->peopleSpawnLocation;
            msg.person_detection_range     = m_currentConfig->personIdentificationRange;
            msg.sim_start_time             = m_currentConfig->simStartTime;
            msg.sim_duration               = m_currentConfig->simDuration;
            msg.battery_voltage            = m_currentConfig->batteryVoltage;
            msg.cv_threshold               = m_currentConfig->cvThreshold;
            msg.taper_fraction             = m_currentConfig->taperFraction;
            msg.charge_to_full             = m_currentConfig->chargeToFull;
            msg.always_charge_at_dock      = m_currentConfig->alwaysChargeAtDock;
        }
        m_publisher->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Simulation configuration published!");
    }

    rclcpp::Service<event_system_msgs::srv::SetSystemConfig>::SharedPtr m_subscription;
    rclcpp::Publisher<event_system_msgs::msg::SystemConfig>::SharedPtr m_publisher;
    std::mutex m_configMutex;
    bool m_dirtyConfig{};
    std::shared_ptr<SimConfig> m_currentConfig;
};

}  // namespace des
