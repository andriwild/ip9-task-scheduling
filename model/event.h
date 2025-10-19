#pragma once

#include "robot.h"
#include "../datastructure/event_queue.h"
#include "../util/util.h"


class RobotDriveEndEvent final : public SimulationEvent {
    Robot *m_robot;
    Node destination;

public:
    RobotDriveEndEvent(const int t, Robot *robot, const Node &dest):
    SimulationEvent(t),
    m_robot(robot),
    destination(dest)
    {}

    void execute(EventQueue &eventQueue) override {
        Log::d("Robot ends driving");
        m_robot->setPosition(destination);
        m_robot->stopDriving();
    }
};


class RobotDriveStartEvent final : public SimulationEvent {
    Robot *m_robot;
    Node destination;

public:
    RobotDriveStartEvent(const int t, Robot *robot, const Node &dest):
    SimulationEvent(t),
    m_robot(robot),
    destination(dest)
    {}

    void execute(EventQueue &eventQueue) override {
        m_robot->startDriving(time);
        m_robot->setTarget(destination);
        int driveTime = util::calcDriveTime(m_robot->getPosition(), destination, m_robot->getSpeed());
        eventQueue.addEvent<RobotDriveEndEvent>(driveTime, m_robot, destination);
        Log::d("Robot starts driving");
    }
};

class PersonEscortEvent final : public SimulationEvent {
    Node m_destination;
    int m_personId;

public:
    PersonEscortEvent(const int t, const Node &dest, const int personId):
    SimulationEvent(t),
    m_destination(dest),
    m_personId(personId)
    {}

    void execute(EventQueue &eventQueue) override {
        Log::d("PersonEscortEvent");
    }
};