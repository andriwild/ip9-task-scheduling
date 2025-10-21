#pragma once

#include "robot.h"
#include "../datastructure/event_queue.h"
#include "../util/util.h"



class RobotDriveStartEvent;


class Tour final : public SimulationEvent {

public:
    Tour(): SimulationEvent(0) {}

    std::string getName() const override {
        return "Tour";
    }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Start Tour");
    }
};

class RobotDriveEndEvent final : public SimulationEvent {
    int m_destinationId;

public:
    RobotDriveEndEvent(const int time, const int destinationId):
    SimulationEvent(time),
    m_destinationId(destinationId)
    {}

    std::string getName() const override {
        return "Drive End Event";
    }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Robot ends driving");
        robot.stopDriving();
        robot.setPosition(m_destinationId);
    }
};


class RobotDriveStartEvent final : public SimulationEvent {
public:
    const int m_targetId;
    const ROBOT_TASK m_task;
    const int m_expectedArrival;

    RobotDriveStartEvent(
        const int time,
        const int expectedArrival,
        const int targetId,
        const ROBOT_TASK task = DRIVE
        ):
    SimulationEvent(time),
    m_targetId(targetId),
    m_task(task),
    m_expectedArrival(expectedArrival)
    {}

    std::string getName() const override {
        return "Drive Start Event";
    }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        robot.startDriving(m_time);
        robot.setTarget(m_targetId);
        robot.setTask(m_task);
        //eventQueue.addEvent<RobotDriveEndEvent>(m_robot, m_expectedArrival, m_targetId);
        Log::d("Robot starts driving");
    }
};

class MeetingEvent final : public SimulationEvent {
    int m_destinationId;
    int m_personId;
    std::vector<RobotDriveStartEvent*> children;

public:
    MeetingEvent(const int time, const int destinationId, const int personId):
    SimulationEvent(time),
    m_destinationId(destinationId),
    m_personId(personId)
    {}

    std::string getName() const override {
        return "Meeting Event [" + std::to_string(m_id) + "]";
    }

    int getPersonId() const {
        return m_personId;
    }

    int getDestination() const {
        return m_destinationId;
    }

    void addDriveEvent(RobotDriveStartEvent *ev) {
        children.push_back(ev);
    }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("PersonEscortEvent");
    }
};
