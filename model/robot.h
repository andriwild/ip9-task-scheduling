#pragma once

#include "../algo/rnd.h"
#include "../datastructure/graph.h"


constexpr double energyUsageStandby = 0.2;
constexpr double energyUsageDrive = 0.4;

class Robot {
    Node m_dock;
    Node m_position;
    std::optional<Node> m_target = std::nullopt;
    double m_speed;
    bool m_isDriving;
    int m_startDrivingTime;
    double m_energy;

public:
    explicit Robot(const Node position, const Node dock, double speed = 3.0, double energy = 100.0):
    m_position(position),
    m_speed(speed),
    m_isDriving(false),
    m_startDrivingTime(0),
    m_energy(energy),
    m_dock(dock)
    {}

    void setPosition(const Node p) {
       m_position = p;
    }

    Node getPosition() const {
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

    Node getDock() {
        return m_dock;
    }

    std::optional<Node> getTarget() const {
        return m_target;
    }

    void setTarget(const std::optional<Node> &target) {
        this->m_target = target;
    }

    friend std::ostream& operator<<(std::ostream& out, const Robot& robot) {
        out << "Robot (" << robot.m_position.x << ", " << robot.m_position.y << std::endl;
        return out;
    }
};