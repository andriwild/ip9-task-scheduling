#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "robot_state.h"

class Robot {
    std::string m_idleLocation = "IMVS_Dock";
    std::unique_ptr<RobotState> m_state;
    double m_currentSpeed = 0;
    double m_accompanySpeed;
    std::string m_currentLocation;
    rclcpp::Logger m_logger;

public:
    Robot(
        const double speed,
        const double accompanySpeed,
        //const std::string& idleLocation,
        rclcpp::Logger logger
    ) :
        //m_idleLocation(idleLocation),
        m_state(std::make_unique<IdleState>()),
        m_currentSpeed(speed),
        m_accompanySpeed(accompanySpeed),
        m_logger(logger) 
    {}


    std::string getLocation() const;
    void setLocation(std::string location);

    void changeState(std::unique_ptr<RobotState> newState);
    RobotState* getState() { return m_state.get(); };

    double getSpeed() const { return m_currentSpeed; };
    void setSpeed(double newSpeed) { m_currentSpeed = newSpeed; }

    std::string getIdleLocation() const { return m_idleLocation; }
    void setIdleLocation(std::string& location) { m_idleLocation = location; }

    double getAccompanySpeed() const { return m_accompanySpeed; }
    void setAccompanytSpeed(double speed) { m_accompanySpeed = speed; }

    bool isBusy();
    bool isSearching();
    bool isAccompany();
    bool isConversate();

    rclcpp::Logger getLogger() const { return m_logger; }
};
