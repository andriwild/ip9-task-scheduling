#pragma once

#include <format>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "../util/types.h"

class ISimContext;
constexpr int ONE_HOUR = 3600;

class IEvent {
public:
    int time;
    explicit IEvent(const int time) : time(time) {}
    virtual ~IEvent() = default;

    virtual void execute(ISimContext& ctx) = 0;
    virtual std::string getName() const = 0;
    virtual des::EventType getType() const = 0;
    virtual std::string getColor() const { return ""; }

    bool operator<(const IEvent& other) const {
        return time < other.time;
    }

    friend std::ostream& operator<<(std::ostream& os, const IEvent& event) {
        os << "[" << event.time << "] " << event.getName();
        return os;
    }
};

struct EventComparator {
    bool operator()(const std::shared_ptr<IEvent>& a, const std::shared_ptr<IEvent>& b) const {
        if (!a || !b) {
            return false;
        }
        return a->time > b->time;
    }
};

using SortedEventQueue = std::priority_queue<
    std::shared_ptr<IEvent>,
    std::vector<std::shared_ptr<IEvent>>,
    EventComparator>;

class SimulationStartEvent final : public IEvent {
public:
    explicit SimulationStartEvent(const int time) : IEvent(time) {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Simulation Start"; }
    des::EventType getType() const override { return des::EventType::SIMULATION_START; }
};

class SimulationEndEvent final : public IEvent {
public:
    explicit SimulationEndEvent(const int time) : IEvent(time) {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Simulation End"; };
    des::EventType getType() const override { return des::EventType::SIMULATION_END; }
};

class StartDriveEvent final : public IEvent {
    std::string location;
public:
    explicit StartDriveEvent(const int time, std::string location)
        : IEvent(time)
        , location(std::move(location))
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Departing: " + location; }
    des::EventType getType() const override { return des::EventType::START_DRIVE; }
};

class StopDriveEvent final : public IEvent {
public:
    double distance;
    std::string location;

    explicit StopDriveEvent(const int time, const std::string &location, const double distance)
        : IEvent(time)
        , distance(distance)
        , location(location) 
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Arrived: " + location; }
    des::EventType getType() const override { return des::EventType::STOP_DRIVE; }
};

class MissionDispatchEvent final : public IEvent {
public:
    std::shared_ptr<des::Appointment> appointment;
    explicit MissionDispatchEvent(const int time, const std::shared_ptr<des::Appointment> &appointment)
        : IEvent(time)
        , appointment(appointment)
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override {
        return std::format("Mission {} Dispatch: {}", appointment->id, appointment->personName);
    }
    des::EventType getType() const override { return des::EventType::MISSION_DISPATCH; }
};

class MissionCompleteEvent final : public IEvent {
    std::shared_ptr<des::Appointment> appointment;
public:
    explicit MissionCompleteEvent(const int time, const std::shared_ptr<des::Appointment> &appt)
        : IEvent(time)
        , appointment(appt)
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override {
        return std::format("Mission {} Complete: {}", appointment->id, des::missionStateStr(appointment->state));
    }
    des::EventType getType() const override { return des::EventType::MISSION_COMPLETE; };
};

class FoundPersonConversationCompleteEvent : public IEvent {
public:
    explicit FoundPersonConversationCompleteEvent(const int time) : IEvent(time) {}
    std::string getName() const override { return "Conversation complete"; }
    des::EventType getType() const override { return des::EventType::FOUND_PERSON_CONV_COMPLETE; };
};

class SuccessFoundPersonConversationCompleteEvent final: public FoundPersonConversationCompleteEvent {
public:
    explicit SuccessFoundPersonConversationCompleteEvent(const int time)
        : FoundPersonConversationCompleteEvent(time)
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Conversation Successful"; }
};

class FailedFoundPersonConversationCompleteEvent final: public FoundPersonConversationCompleteEvent {
public:
    explicit FailedFoundPersonConversationCompleteEvent(const int time)
        : FoundPersonConversationCompleteEvent(time)
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Conversation Failed "; }
};

class DropOffConversationCompleteEvent : public IEvent {
public:
    explicit DropOffConversationCompleteEvent(const int time) : IEvent(time) {}
    std::string getName() const override { return "Conversation complete" ; }
    des::EventType getType() const override { return des::EventType::DROP_OFF_CONV_COMPLETE; };
};

class SuccessDropOffConversationCompleteEvent final: public DropOffConversationCompleteEvent {
public:
    explicit SuccessDropOffConversationCompleteEvent(const int time)
        : DropOffConversationCompleteEvent(time)
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Conversation Successful"; }
};

class FailedDropOffConversationCompleteEvent final: public DropOffConversationCompleteEvent {
public:
    explicit FailedDropOffConversationCompleteEvent(const int time)
        : DropOffConversationCompleteEvent(time)
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Conversation Failed "; }
};

class AbortSearchEvent final : public IEvent {
public:
    explicit AbortSearchEvent(const int time) : IEvent(time) {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Abort Search"; }
    des::EventType getType() const override { return des::EventType::ABORT_SEARCH; }
};


class StartDropOffConversationEvent final : public IEvent {
public:
    explicit StartDropOffConversationEvent(const int time) : IEvent(time) {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Start Drop Off Conversation"; }
    des::EventType getType() const override { return des::EventType::START_DROP_OFF_CONV; }
};


class StartFoundPersonConversationEvent final : public IEvent {
public:
    explicit StartFoundPersonConversationEvent(const int time) : IEvent(time) {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Found Person Conversation"; }
    des::EventType getType() const override { return des::EventType::START_FOUND_PERSON_CONV; }
};


class StartAccompanyEvent final : public IEvent {
public:
    explicit StartAccompanyEvent(const int time) : IEvent(time) {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Start Accompany"; }
    des::EventType getType() const override { return des::EventType::START_ACCOMPANY; }
};

class BatteryFullEvent final : public IEvent {
public:
    explicit BatteryFullEvent(const int time) : IEvent(time) {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { return "Battery Full"; }
    des::EventType getType() const override { return des::EventType::BATTERY_FULL; }
};

class MissionStartEvent final : public IEvent {
public:
    std::shared_ptr<des::Appointment> appointment;
    explicit MissionStartEvent(const int time, const std::shared_ptr<des::Appointment> &appointment)
        : IEvent(time)
        , appointment(appointment)
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override {
        return std::format("Mission {} Start", appointment->id);
    }
    des::EventType getType() const override { return des::EventType::MISSION_START; }
};

class PersonTransitionEvent: public IEvent {
public:
    const std::shared_ptr<des::Person> person;
    explicit PersonTransitionEvent(const int time, std::shared_ptr<des::Person> p) :
        IEvent(time), 
        person(p) 
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { 
        return std::format("{} moved to {}", person->firstName, person->currentRoom);
 }
    des::EventType getType() const override { return des::EventType::PERSON_TRANSITION; }
    std::string getColor() const override { return person->color; }
};

class PersonArrivedEvent final : public PersonTransitionEvent {
public:
    explicit PersonArrivedEvent(const int time, std::shared_ptr<des::Person> p) :
        PersonTransitionEvent(time, p)
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { 
        return std::format("{} arrived to {}", person->firstName, person->currentRoom);
 }
    des::EventType getType() const override { return des::EventType::PERSON_ARRIVED; }
};

class PersonDepartureEvent final : public PersonTransitionEvent {
public:
    explicit PersonDepartureEvent(const int time, std::shared_ptr<des::Person> p) : 
    PersonTransitionEvent(time, p)
    {}
    void execute(ISimContext& ctx) override;
    std::string getName() const override { 
        return std::format("{} leaved to {}", person->firstName, person->currentRoom);
 }
    des::EventType getType() const override { return des::EventType::PERSON_DEPARTURE; }
};
