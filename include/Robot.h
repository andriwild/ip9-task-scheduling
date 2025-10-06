#pragma once

#include "Event.h"

class Robot {
    Point m_location;
    double m_speed;
    bool m_isDriving;
    int m_startDrivingTime;

public:
    explicit Robot(const Point p, double speed = 3.0) :
    m_location(p),
    m_speed(speed),
    m_isDriving(false),
    m_startDrivingTime(0)
    {}

    void setPosition(const Point p) {
       m_location = p;
    }

    Point getPosition() const {
        return m_location;
    }

    double getSpeed() const {
        return m_speed;
    }

    void setSpeed(const double speed) {
        m_speed = speed;
    }

    bool isDriving() const {
        return m_isDriving;
    }

    void setDriving(const int time) {
        m_isDriving = true;
        m_startDrivingTime = time;
    }

    void stopDriving() {
        m_isDriving = false;
    }

    int getStartDrivingTime() const {
        return m_startDrivingTime;
    }

    friend std::ostream& operator<<(std::ostream& out, const Robot& robot) {
        out << "Robot (" << robot.m_location.x << ", " << robot.m_location.y << std::endl;
        return out;
    }
};
