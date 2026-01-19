#pragma once

#include <memory>

#include "robot_state.h"

class Robot {
    std::string m_idleLocation = "IMVS_Dock";
    std::unique_ptr<RobotState> m_state;
    double m_currentSpeed = 0;
    double m_accompanySpeed;
    double m_energy = 100.0;
    std::string m_currentLocation;

public:
    Robot(const double speed, const double accompanySpeed):
        m_state(std::make_unique<IdleState>()),
        m_currentSpeed(speed),
        m_accompanySpeed(accompanySpeed)
    {}

    void setAccompanytSpeed(double speed) { m_accompanySpeed = speed; }

    std::string getLocation() const;
    void setLocation(std::string location);

    void changeState(std::unique_ptr<RobotState> newState);
    RobotState* getState() { return m_state.get(); };

    void setSpeed(double newSpeed);
    double getSpeed() const;

    std::string getIdleLocation() const {
        return m_idleLocation;
    }

    double getAccompanySpeed() const { return m_accompanySpeed; }

    bool isBusy();
    bool isSearching();
    bool isAccompany();
    bool isConversate();
};
