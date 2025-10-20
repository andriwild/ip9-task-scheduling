#pragma once

#include "robot.h"
#include "../datastructure/event_queue.h"
#include "../util/util.h"


class RobotDriveEndEvent final : public SimulationEvent {
    Robot *m_robot;
    int m_destinationId;

public:
    RobotDriveEndEvent(Robot *robot, const int time, const int destinationId):
    SimulationEvent(time),
    m_robot(robot),
    m_destinationId(destinationId)
    {}

    void execute(EventQueue &eventQueue, bool randomness = true) override {
        Log::d("Robot ends driving");
        m_robot->stopDriving();
        m_robot->setPosition(m_destinationId);
    }
};

class RobotDriveStartEvent final : public SimulationEvent {
    Robot *m_robot;

public:
    const int m_targetId;
    const ROBOT_TASK m_task;
    const int m_expectedArrival;

    RobotDriveStartEvent(
        Robot *robot,
        const int time,
        const int expectedArrival,
        const int targetId,
        const ROBOT_TASK task = DRIVE
        ):
    SimulationEvent(time),
    m_robot(robot),
    m_targetId(targetId),
    m_task(task),
    m_expectedArrival(expectedArrival)
    {}

    void execute(EventQueue &eventQueue, bool randomness = true) override {
        m_robot->startDriving(time);
        m_robot->setTarget(m_targetId);
        m_robot->setTask(m_task);
        //eventQueue.addEvent<RobotDriveEndEvent>(m_robot, time + m_expectedArrival, m_targetId);
        Log::d("Robot starts driving");
    }
};

class MeetingEvent final : public SimulationEvent {
    int m_destinationId;
    int m_personId;

public:
    MeetingEvent(const int time, const int destinationId, const int personId):
    SimulationEvent(time),
    m_destinationId(destinationId),
    m_personId(personId)
    {}

    int getPersonId() const {
        return m_personId;
    }

    int getDestination() const {
        return m_destinationId;
    }

    void execute(EventQueue &eventQueue, bool randomness = true) override {
        Log::d("PersonEscortEvent");
    }
};
