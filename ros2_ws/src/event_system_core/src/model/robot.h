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

    double m_driveSpeed;
    double m_accompanySpeed;
    double m_currentSpeed = 0;

    bool m_isDriving        = false;
    bool m_isCharging       = false;
    bool m_chargingRequired = false;

public:
    std::unique_ptr<Battery> m_bat;

    explicit Robot(const std::shared_ptr<des::SimConfig> &config)
        : m_state(std::make_unique<IdleState>())
    {
        m_driveSpeed     = config->robotSpeed;
        m_accompanySpeed = config->robotAccompanySpeed;

        m_bat = std::make_unique<Battery>(
            config->batteryCapacity,
            config->initialBatteryCapacity,
            config->lowBatteryThreshold,
            config->fullBatteryThreshold
        );
    }

    bool isBusy() const;
    void updateConfig(const des::SimConfig& config);

    std::string getLocation() const { return m_currentLocation; };
    void setLocation(const std::string &location) { 
        m_currentLocation = location; 
        RCLCPP_DEBUG(rclcpp::get_logger("Robot"), "Robot location set to: %s", location.c_str());
    }

    std::string getTargetLocation() const { return m_targetLocation; }
    void setTargetLocation(const std::string &location) { 
        m_targetLocation = location; 
        RCLCPP_DEBUG(rclcpp::get_logger("Robot"), "Robot target location set to: %s", location.c_str());
    }

    void changeState(std::unique_ptr<RobotState> newState);
    RobotState* getState() const { return m_state.get(); }

    bool isDriving() const { return m_isDriving; }
    void setDriving(const bool isDriving) { 
        m_isDriving = isDriving; 
    }

    bool isCharging() const { return m_isCharging; }
    void setCharging(const bool isCharging) { 
        m_isCharging = isCharging; 
    }

    bool updateAndGetChargingRequired() {
        m_chargingRequired = false;
        if (m_bat->isBatteryLow()) {
            m_chargingRequired = true;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("Robot"), "Robot charging required: %d", m_chargingRequired);
        return m_chargingRequired;
    }
    void setChargingRequired(const bool isChargingRequired) { 
        m_chargingRequired = isChargingRequired; 
    }

    des::RobotStateType getStateType() const { return m_state->getType(); }

    double getCurrentSpeed() const { return m_currentSpeed; }
    void setSpeed(const double newSpeed) { 
        m_currentSpeed = newSpeed; 
        RCLCPP_DEBUG(rclcpp::get_logger("Robot"), "Robot speed set to: %.2f", newSpeed);
    }

    double getAccompanySpeed() const { return m_accompanySpeed; }
    void setAccompanySpeed(const double speed) { 
        m_accompanySpeed = speed; 
        RCLCPP_DEBUG(rclcpp::get_logger("Robot"), "Robot accompany speed set to: %.2f", speed);
    }

    double getDriveSpeed() const { return m_driveSpeed; }
    void setDriveSpeed(const double speed) { 
        m_driveSpeed = speed; 
        RCLCPP_DEBUG(rclcpp::get_logger("Robot"), "Robot drive speed set to: %.2f", speed);
    }

    std::string getIdleLocation() const { return m_dockLocation; }
    void setIdleLocation(const std::string &location) { 
        m_dockLocation = location; 
        RCLCPP_DEBUG(rclcpp::get_logger("Robot"), "Robot idle/dock location set to: %s", location.c_str());
    }
};