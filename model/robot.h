#pragma once

#include "../algo/rnd.h"

enum ROBOT_TASK { IDLE, DRIVE, ESCORT, SEARCH };

constexpr double energyUsageStandby = 0.2;
constexpr double energyUsageDrive = 0.4;

class Robot {
    int m_dock;
    int m_position;
    int  m_target = -1;
    double m_speed;
    bool m_isDriving;
    int m_startDrivingTime;
    double m_energy;
    ROBOT_TASK m_task = IDLE;

public:
    explicit Robot(const int position, const int dock, double speed = 3.0, double energy = 100.0):
    m_position(position),
    m_speed(speed),
    m_isDriving(false),
    m_startDrivingTime(0),
    m_energy(energy),
    m_dock(dock)
    {}

    void setPosition(const int p) {
       m_position = p;
    }

    int getPosition() const {
        return m_position;
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

    void startDriving(const int time) {
        m_isDriving = true;
        m_startDrivingTime = time;
    }

    void stopDriving() {
        m_isDriving = false;
        m_target = -1;
    }

    int getStartDrivingTime() const {
        return m_startDrivingTime;
    }

    void energyConsumption(int time, bool drive) {
        double usage = energyUsageStandby;
        if (drive) {
            usage = energyUsageDrive;
        }
        m_energy -= time * usage;
    }

    double getEnergy() const {
        return m_energy;
    }

    int getDock() {
        return m_dock;
    }

    int getTarget() const {
        return m_target;
    }

    void setTarget(const int target) {
        this->m_target = target;
    }

    void setTask(ROBOT_TASK task) {
        m_task = task;
    }

    ROBOT_TASK getTask() const {
        return m_task;
    }

    friend std::ostream& operator<<(std::ostream& out, const Robot& robot) {
        out << "Robot at node id: " << robot.m_position  << std::endl;
        return out;
    }
};