#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "robot_state.h"
#include "battery.h"

class Robot {
    std::string m_idleLocation = "IMVS_Dock";
    std::unique_ptr<RobotState> m_state;
    double m_currentSpeed = 0;
    double m_accompanySpeed;
    std::string m_currentLocation;
    std::unique_ptr<Battery> m_bat;
    rclcpp::Logger m_logger;

public:
    Robot(
        std::shared_ptr<des::SimConfig> config,
        rclcpp::Logger logger
    ) :
        m_state(std::make_unique<IdleState>()),
        m_logger(logger)
    {
        m_currentSpeed   = config->robotSpeed;
        m_accompanySpeed = config->robotAccompanySpeed;

        m_bat = std::make_unique<Battery>(
            config->energyConsumptionBase,
            config->energyConsumptionDrive,
            config->batteryCapacity,
            config->chargingRate,
            config->lowBatteryThreshold
        );
    }

    std::string getLocation() const;
    void setLocation(std::string location, double distance);

    void changeState(std::unique_ptr<RobotState> newState);
    RobotState* getState() { return m_state.get(); };

    double getSpeed() const { return m_currentSpeed; };
    void setSpeed(double newSpeed) { m_currentSpeed = newSpeed; }

    std::string getIdleLocation() const { return m_idleLocation; }
    void setIdleLocation(std::string& location) { m_idleLocation = location; }

    double getAccompanySpeed() const { return m_accompanySpeed; }
    void setAccompanytSpeed(double speed) { m_accompanySpeed = speed; }

    void dischargeBattery(int time) { m_bat->discharge(time); }

    void resetBattery() { m_bat.reset(); };

    bool isBusy();
    bool isSearching();
    bool isAccompany();
    bool isConversate();

    rclcpp::Logger getLogger() const { return m_logger; }
};
