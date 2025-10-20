#pragma once

#include "robot.h"
#include "../datastructure/event_queue.h"
#include "../util/util.h"


class RobotDriveEndEvent final : public SimulationEvent {
    Robot *m_robot;
    int m_destinationId;

public:
    RobotDriveEndEvent(Robot *robot, const int t, const int destinationId):
    SimulationEvent(t),
    m_robot(robot),
    m_destinationId(destinationId)
    {}

    void execute(EventQueue &eventQueue) override {
        Log::d("Robot ends driving");
        m_robot->stopDriving();
        m_robot->setPosition(m_destinationId);
    }
};

class RobotDriveStartEvent final : public SimulationEvent {
    Robot *m_robot;
    int m_targetId;

public:
    RobotDriveStartEvent(Robot *robot, const int t, const int targetId):
    SimulationEvent(t),
    m_robot(robot),
    m_targetId(targetId)
    {}

    void execute(EventQueue &eventQueue) override {
        m_robot->startDriving(time);
        m_robot->setTarget(m_targetId);
        // int driveTime = util::calcDriveTime(m_robot->getPosition(), destination, m_robot->getSpeed());
        // eventQueue.addEvent<RobotDriveEndEvent>(driveTime, m_robot, destination);
        Log::d("Robot starts driving");
    }
};

class PersonEscortEvent final : public SimulationEvent {
    int m_destinationId;
    int m_personId;

public:
    PersonEscortEvent(const int t, const int destinationId, const int personId):
    SimulationEvent(t),
    m_destinationId(destinationId),
    m_personId(personId)
    {}

    int getPersonId() const {
        return m_personId;
    }

    int getDestination() const {
        return m_destinationId;
    }

    void execute(EventQueue &eventQueue) override {
        Log::d("PersonEscortEvent");
    }
};