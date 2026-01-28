#pragma once

#include <iostream>
#include <memory>
#include <queue>
#include <vector>

#include "../util/types.h"

class SimulationContext;

class IEvent {
public:
    int time;
    IEvent(const int time) : time(time) {}

    virtual ~IEvent() = default;

    virtual void execute(SimulationContext& ctx) = 0;
    virtual std::string getName() const = 0;
    virtual des::EventType getType() const = 0;

    bool operator<(const IEvent& other) const {
        return time < other.time;
    }

    friend std::ostream& operator<<(std::ostream& os, const IEvent& event) {
        os << "[" << event.time << "] " << event.getName();
        return os;
    }
};

struct EventComparator {
    bool operator()(const std::shared_ptr<IEvent>& a, const std::shared_ptr<IEvent>& b) {
        if (!a || !b) {
            return false;
        }
        return a->time > b->time;
    }
};

using EventQueue = std::priority_queue<
    std::shared_ptr<IEvent>,
    std::vector<std::shared_ptr<IEvent>>,
    EventComparator>;

class SimulationStartEvent : public IEvent {
public:
    explicit SimulationStartEvent(int time) : IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Simulation Start"; };
    des::EventType getType() const override { return des::EventType::SIMULATION_START; };
};

class SimulationEndEvent : public IEvent {
public:
    explicit SimulationEndEvent(int time) : IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Simulation End"; };
    des::EventType getType() const override { return des::EventType::SIMULATION_END; };
};

class ArrivedEvent : public IEvent {
public:
    double distance;
    std::string location;
    explicit ArrivedEvent(int time, std::string location, double distance):
        IEvent(time),
        distance(distance),
        location(location) 
    {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "ArrivedEvent"; };
    des::EventType getType() const override { return des::EventType::ARRIVED; };
};

class MissionDispatchEvent : public IEvent {
public:
    std::shared_ptr<des::Appointment> appointment;
    explicit MissionDispatchEvent(int time, std::shared_ptr<des::Appointment> appt):
        IEvent(time),
        appointment(appt) 
    {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Mission Dispatch"; };
    des::EventType getType() const override { return des::EventType::MISSION_DISPATCH; };
};

class MissionCompleteEvent : public IEvent {
public:
    explicit MissionCompleteEvent(int time) : IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Mission Complete"; };
    des::EventType getType() const override { return des::EventType::MISSION_COMPLETE; };
};

class FoundPersonConversationCompleteEvent : public IEvent {
public:
    explicit FoundPersonConversationCompleteEvent(int time) : IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Found Person Conversation Complete"; };
    des::EventType getType() const override { return des::EventType::FOUND_PERSON_CONV_COMPLETE; };
};

class DropOffConversationCompleteEvent : public IEvent {
public:
    explicit DropOffConversationCompleteEvent(int time) : IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Drop Off Conversation Complete"; };
    des::EventType getType() const override { return des::EventType::DROP_OFF_CONV_COMPLETE; };
};

class AbortSearchEvent: public IEvent {
public:
    explicit AbortSearchEvent(int time) : IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Abort Search"; };
    des::EventType getType() const override { return des::EventType::ABORT_SEARCH; };
};


class StartDropOffConversationeEvent: public IEvent {
public:
    explicit StartDropOffConversationeEvent(int time) : IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Start Drop Off Conversation"; };
    des::EventType getType() const override { return des::EventType::START_DROP_OFF_CONV; };
};


class StartFoundPersonConversationEvent: public IEvent {
public:
    explicit StartFoundPersonConversationEvent(int time) : IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Start Found Person Conversation"; };
    des::EventType getType() const override { return des::EventType::START_FOUND_PERSON_CONV; };
};


class StartAccompanyEvent: public IEvent {
public:
    explicit StartAccompanyEvent(int time) : IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Start Accompany"; };
    des::EventType getType() const override { return des::EventType::START_ACCOMPANY; };
};

