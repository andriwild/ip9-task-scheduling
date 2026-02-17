#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "robot_state.h"
#include "battery.hpp"

class Robot {
    std::string m_dockLocation = "IMVS_Dock";
    std::unique_ptr<RobotState> m_state;
    std::string m_currentLocation;
    std::string m_targetLocation;
    rclcpp::Logger m_logger;

    double m_driveSpeed;
    double m_accompanySpeed;
    double m_currentSpeed = 0;

    bool m_isDriving        = false;
    bool m_isCharging       = false;
    bool m_chargingRequired = false;

public:
    std::unique_ptr<Battery> m_bat;

    Robot(const std::shared_ptr<des::SimConfig> &config, const rclcpp::Logger& logger)
        : m_state(std::make_unique<IdleState>())
        , m_logger(logger)
    {
        m_driveSpeed     = config->robotSpeed;
        m_accompanySpeed = config->robotAccompanySpeed;

        m_bat = std::make_unique<Battery>(
            config->batteryCapacity,
            config->initialBatteryCapacity,
            config->lowBatteryThreshold,
            config->fullBatteryThreshold,
            logger
        );
    }

    bool isBusy() const;
    void updateConfig(const des::SimConfig& config);

    std::string getLocation() const { return m_currentLocation; };
    void setLocation(const std::string &location) { m_currentLocation = location; }

    std::string getTargetLocation() const { return m_targetLocation; }
    void setTargetLocation(const std::string &location) { m_targetLocation = location; }

    void changeState(std::unique_ptr<RobotState> newState);
    RobotState* getState() const { return m_state.get(); }

    bool isDriving() const { return m_isDriving; }
    void setDriving(const bool isDriving) { m_isDriving = isDriving; }

    bool isCharging() const { return m_isCharging; }
    void setCharging(const bool isCharging) { m_isCharging= isCharging; }

    bool isChargingRequired() const { return m_chargingRequired; }
    void setChargingRequired(const bool isChargingRequired) { m_chargingRequired = isChargingRequired; }

    des::RobotStateType getStateType() const { return m_state->getType(); }

    double getCurrentSpeed() const { return m_currentSpeed; }
    void setSpeed(const double newSpeed) { m_currentSpeed = newSpeed; }

    double getAccompanySpeed() const { return m_accompanySpeed; }
    void setAccompanySpeed(const double speed) { m_accompanySpeed = speed; }

    double getDriveSpeed() const { return m_driveSpeed; }
    void setDriveSpeed(const double speed) { m_driveSpeed = speed; }

    std::string getIdleLocation() const { return m_dockLocation; }
    void setIdleLocation(const std::string &location) { m_dockLocation = location; }

    rclcpp::Logger getLogger() const { return m_logger; }
};