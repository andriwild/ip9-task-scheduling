#pragma once

#include <memory>

#include "robot_state.h"

class Robot {
    std::string m_idleLocation = "Dock";
    std::unique_ptr<RobotState> m_state;
    double m_currentSpeed = 0;
    double m_defaultSpeed;
    double m_escortSpeed;
    std::string m_currentLocation;

public:
    Robot(const double defaultSpeed, const double escortSpeed):
        m_defaultSpeed(defaultSpeed),
        m_escortSpeed(escortSpeed),
        m_state(std::make_unique<IdleState>())
    {}

    std::string getLocation() const;
    void setLocation(std::string location);

    void changeState(RobotState* newState);
    RobotState* getState() { return m_state.get(); };

    void setSpeed(double newSpeed);
    double getSpeed() const;

    std::string getIdleLocation() const {
        return m_idleLocation;
    }

    double getDefaultSpeed() const { return m_defaultSpeed; }
    double getEscortSpeed() const { return m_escortSpeed; }

    bool isBusy();
    bool isSearching();
    bool isEscorting();
};
