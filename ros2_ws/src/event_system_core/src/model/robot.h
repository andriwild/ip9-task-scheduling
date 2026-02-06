#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "robot_state.h"
#include "battery.h"

class Robot {
    std::string m_idleLocation = "IMVS_Dock";
    std::unique_ptr<RobotState> m_state;
    std::string m_currentLocation;
    rclcpp::Logger m_logger;

    double m_driveSpeed;
    double m_accompanySpeed;
    double m_currentSpeed = 0;

public:
    std::unique_ptr<Battery> m_bat;

    Robot(
        std::shared_ptr<des::SimConfig> config,
        rclcpp::Logger logger
    ) :
        m_state(std::make_unique<IdleState>()),
        m_logger(logger)
    {
        m_driveSpeed = config->robotSpeed;
        m_accompanySpeed = config->robotAccompanySpeed;

        m_bat = std::make_unique<Battery>(
            config->batteryCapacity,
            config->initialBatteryCapacity, // TODO: not really worth to store in battery class
            config->lowBatteryThreshold
        );
    }

    bool isBusy() const;
    void updateConfig(des::SimConfig& config);

    std::string getLocation() const;
    void setLocation(std::string location);

    void changeState(std::unique_ptr<RobotState> newState);
    RobotState* getState() { return m_state.get(); };

    des::RobotStateType getStateType() const { return m_state->getType(); };

    double getCurrentSpeed() const { return m_currentSpeed; };
    void setSpeed(double newSpeed) { m_currentSpeed = newSpeed; }

    double getAccompanySpeed() const { return m_accompanySpeed; }
    void setAccompanytSpeed(double speed) { m_accompanySpeed = speed; }

    double getDriveSpeed() const { return m_driveSpeed; }
    void setDriveSpeed(double speed) { m_driveSpeed = speed; }

    std::string getIdleLocation() const { return m_idleLocation; }
    void setIdleLocation(std::string& location) { m_idleLocation = location; }

    rclcpp::Logger getLogger() const { return m_logger; }
};
