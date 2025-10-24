#pragma once

#include <iomanip>

#include "robot.h"
#include "../datastructure/tree.h"
#include "../util/util.h"
#include "../view/helper.h"


inline static int nextId;

const std::string defaultColor = "\033[0m";

class SimulationEvent {
protected:
    int m_id;
    int m_time;
    std::string m_label;
    std::string m_color;

public:
    explicit SimulationEvent(const int t, const std::string &color = defaultColor, const std::string& label = ""):
    m_id(nextId++), m_time(t), m_label(label), m_color(color)
    {}

    virtual ~SimulationEvent() = default;

    virtual void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) = 0;
    int getId() const { return m_id; }
    int getTime() const { return m_time; };
    virtual void setTime(const int time) { m_time = time; };
    virtual std::string getName() const  = 0;
    std::string getLabel() const { return m_label; }

    bool operator<(const SimulationEvent& other) {
        return m_time < other.m_time;
    }

    friend std::ostream& operator<<(std::ostream& os, const SimulationEvent& event) {
        os << event.m_color;
        os << "[" << event.getId() << "] "
           << std::left << std::setw(20) << event.getName()
           << " (t=" << event.m_time << ")  "
           << std::setw(15) << event.getLabel();
        os << defaultColor;
        return os;
    }
};

class SimulationRoot final : public SimulationEvent {
public:
    explicit SimulationRoot(const int time, const std::string& label = ""): SimulationEvent(time, label) {}

    std::string getName() const override { return "Simulation"; }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Simulation");
    }
};

class Tour final : public SimulationEvent {
public:
    explicit Tour(const int time, const std::string& label = ""):
    SimulationEvent(time, Helper::colorAnsi(TOUR), label) { }

    std::string getName() const override { return "Tour"; }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Start Tour");
    }
};

class RobotDriveEndEvent final : public SimulationEvent {
    int m_destinationId;

public:
    RobotDriveEndEvent(const int time, const int destinationId):
    SimulationEvent(time, Helper::colorAnsi(DRIVE)),
    m_destinationId(destinationId)
    { }

    std::string getName() const override { return "Drive End Event"; }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Robot ends driving");
        robot.stopDriving();
        robot.setPosition(m_destinationId);
    }
};

class RobotDriveStartEvent final : public SimulationEvent {
public:
    const int m_targetId;
    const TYPE task;

    RobotDriveStartEvent(
        const int time,
        const int targetId,
        const TYPE task = DRIVE,
        const std::string& label = ""
        ):
    SimulationEvent(time, Helper::colorAnsi(DRIVE) ,label),
    m_targetId(targetId),
    task(task)
    { }

    std::string getName() const override { return "Drive Start Event"; }

    int getTarget() const { return m_targetId; }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        robot.startDriving(m_time);
        robot.setTarget(m_targetId);
        robot.setTask(task);
        Log::d("Robot starts driving");
    }
};

class MeetingEvent final : public SimulationEvent {
    int m_destinationId;
    int m_personId;

public:
    MeetingEvent(const int time, const int destinationId, const int personId, const std::string& label = ""):
    SimulationEvent(time, Helper::colorAnsi(MEETING) ,label),
    m_destinationId(destinationId),
    m_personId(personId)
    { }

    std::string getName() const override { return "Meeting Event [" + std::to_string(m_id) + "]"; }
    int getPersonId() const { return m_personId; }
    int getDestination() const { return m_destinationId; }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Meeting event");
    }
};

class SearchEvent final : public SimulationEvent {
    const int m_personId;

public:
    SearchEvent(
        const int time,
        const int personId,
        const std::string& label = ""
        ):
    SimulationEvent(time, Helper::colorAnsi(SEARCH) ,label),
    m_personId(personId)
    { }

    int getSearchForPersonId() const { return m_personId; }

    std::string getName() const override { return "Search for Person: " + std::to_string(m_personId) ; }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Searching ...");
    }
};

class TalkingEvent : public SimulationEvent {
    const int m_personId;

public:
    TalkingEvent(const int time, const int personId, const std::string& label = "" ):
    SimulationEvent(time, Helper::colorAnsi(TALK) ,label),
    m_personId(personId)
    { }

    int getSearchForPersonId() const { return m_personId; }

    std::string getName() const override { return "Talking with Person: " + std::to_string(m_personId) ; }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Talking ...");
    }
};

class TalkingEventStart : public TalkingEvent {
public:
    TalkingEventStart(const int time, const int personId, const std::string& label = "" ):
    TalkingEvent(time, personId, label) {}

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Talking start ...");
    }
};

class TalkingEventEnd: public TalkingEvent {
public:
    TalkingEventEnd(const int time, const int personId, const std::string& label = "" ):
    TalkingEvent(time, personId, label) {}

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Talking end...");
    }
};

class EscortEvent final : public SimulationEvent {
    const int m_personId;
public:
    EscortEvent( const int time, const int personId, const std::string& label = "" ):
    SimulationEvent(time, Helper::colorAnsi(ESCORT) ,label),
    m_personId(personId)
    { }

    std::string getName() const override { return "Escort Person: " + std::to_string(m_personId) ; }

    void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) override {
        Log::d("Escorting...");
    }
};
