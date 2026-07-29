#pragma once

#include <algorithm>
#include <format>
#include <iterator>
#include <utility>

#include "base.h"
#include "../i_sim_context.h"
#include "../occupancy.h"
#include "../../util/rnd.h"

class PersonDepartureEvent;

inline int busyRetryAt(ISimContext& ctx, const int now) {
    return now + static_cast<int>(rnd::uni(ctx.robotRng(), 60, 300));
}

inline double personWalkTime(ISimContext& ctx, des::Person& person, const std::string& from, const std::string& to) {
    const std::optional<double> dist = ctx.getDistance(from, to);
    const double speed = ctx.getConfig()->personSpeed;
    if (!dist.has_value() || speed <= 0.0) {
        return 0.0;
    }
    const double t = dist.value() / speed;
    const double tRnd = rnd::normal(person.rng, t, t * 0.1);
    return tRnd < 0 ? t : tRnd;
}

class PersonTransitionEvent : public IEvent {
public:
    des::Person* const person;
    std::string targetRoom;
    explicit PersonTransitionEvent(const int time, des::Person* p) :
        IEvent(time),
        person(std::move(p))
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<PersonTransitionEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override;

    std::string getName() const override {
        return std::format("{} walking to {}", person->firstName, targetRoom);
    }
    des::EventType getType() const override { return des::EventType::PERSON_TRANSITION; }
    std::string getColor() const override { return person->color; }
};

class PersonArrivedEvent final : public PersonTransitionEvent {
public:
    explicit PersonArrivedEvent(const int time, des::Person* p) :
        PersonTransitionEvent(time, std::move(p))
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<PersonArrivedEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& p = *this->person;

        if (p.busy) {
            ctx.pushEvent(this->withTime(busyRetryAt(ctx, this->time)));
            return;
        }

        const std::string currentRoom = ctx.getPersonLocation(p.firstName);
        targetRoom = currentRoom;
        ctx.notifyEvent(*this);

        auto it = std::find(p.roomLabels.begin(), p.roomLabels.end(), currentRoom);

        // Person arrived at unknown room (e.g. after accompany) — return to workplace
        if (it == p.roomLabels.end()) {
            ctx.setPersonLocation(p.firstName, p.workplace);
            ctx.pushEvent(std::make_shared<PersonTransitionEvent>(
                this->time + rnd::uni(p.rng, 60, ONE_HOUR), this->person));
            return;
        }

        int currentIndex = std::distance(p.roomLabels.begin(), it);
        const std::vector<double>& row = p.transitionMatrix.at(currentIndex);
        int nextRoomIdx = rnd::discrete_dist(p.rng, row);

        ctx.setPersonLocation(p.firstName, p.roomLabels.at(nextRoomIdx));
        double nextExecutionTime = this->time + rnd::uni(p.rng, 10, 30);
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(nextExecutionTime, this->person));
    }

    std::string getName() const override {
        return std::format("{} arrived to {}", person->firstName, targetRoom);
    }
    des::EventType getType() const override { return des::EventType::PERSON_ARRIVED; }
};

class PersonDepartureEvent final : public PersonTransitionEvent {
public:
    explicit PersonDepartureEvent(const int time, des::Person* p) :
        PersonTransitionEvent(time, std::move(p))
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<PersonDepartureEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        // accompany guard
        if (this->person->busy) {
            ctx.pushEvent(this->withTime(busyRetryAt(ctx, this->time)));
            return;
        }
        targetRoom = "OUTDOOR";
        ctx.setPersonLocation(this->person->firstName, "OUTDOOR");
        ctx.notifyEvent(*this);

        const int nextDayBase = (this->time / SECONDS_PER_DAY + 1) * SECONDS_PER_DAY;
        des::sampleOccupancy(*ctx.getConfig(), this->person->rng, nextDayBase, *this->person);
        const auto simEnd = ctx.getSimulationEndTime();
        if (simEnd.has_value() && this->person->arrivalTime < simEnd.value()) {
            ctx.pushEvent(std::make_shared<PersonArrivedEvent>(this->person->arrivalTime, this->person));
        }
    }

    std::string getName() const override {
        return std::format("{} leaved to {}", person->firstName, targetRoom);
    }
    des::EventType getType() const override { return des::EventType::PERSON_DEPARTURE; }
};

class PersonLunchArrivedEvent final : public PersonTransitionEvent {
public:
    explicit PersonLunchArrivedEvent(const int time, des::Person* p, std::string room) :
        PersonTransitionEvent(time, std::move(p))
    {
        targetRoom = std::move(room);
    }

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<PersonLunchArrivedEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& p = *this->person;

        if (p.busy) {
            ctx.notifyEvent(*this);
            ctx.pushEvent(std::make_shared<PersonTransitionEvent>(busyRetryAt(ctx, this->time), this->person));
            return;
        }

        ctx.setPersonLocation(p.firstName, targetRoom);
        ctx.notifyEvent(*this);
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(
            static_cast<int>(this->time + p.lunchDuration), this->person));
    }

    std::string getName() const override {
        return std::format("{} having lunch at {}", person->firstName, targetRoom);
    }
    des::EventType getType() const override { return des::EventType::PERSON_ROOM_ARRIVED; }
};

class PersonLunchEvent final : public PersonTransitionEvent {
public:
    explicit PersonLunchEvent(const int time, des::Person* p, std::string room) :
        PersonTransitionEvent(time, std::move(p))
    {
        targetRoom = std::move(room);
    }

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<PersonLunchEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& p = *this->person;

        if (p.busy) {
            ctx.pushEvent(this->withTime(busyRetryAt(ctx, this->time)));
            return;
        }

        const std::string currentRoom = ctx.getPersonLocation(p.firstName);
        ctx.notifyEvent(*this);

        const double walkTime = personWalkTime(ctx, p, currentRoom, targetRoom);
        ctx.setPersonLocation(p.firstName, IN_TRANSIT);
        ctx.pushEvent(std::make_shared<PersonLunchArrivedEvent>(
            static_cast<int>(this->time + walkTime), this->person, targetRoom));
    }

    des::EventType getType() const override { return des::EventType::PERSON_TRANSITION; }
};

class PersonRoomArrivedEvent final : public PersonTransitionEvent {
public:
    explicit PersonRoomArrivedEvent(const int time, des::Person* p, std::string room) :
        PersonTransitionEvent(time, std::move(p))
    {
        targetRoom = std::move(room);
    }

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<PersonRoomArrivedEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& p = *this->person;

        // accompany guard
        if (p.busy) {
            ctx.notifyEvent(*this);
            ctx.pushEvent(std::make_shared<PersonTransitionEvent>(busyRetryAt(ctx, this->time), this->person));
            return;
        }

        ctx.setPersonLocation(p.firstName, targetRoom);
        ctx.notifyEvent(*this);

        double nextExecutionTime = this->time + p.getStayDuration(targetRoom, p.rng);

        if (p.lunchPending && p.lunchTime < nextExecutionTime && p.lunchTime < p.departureTime) {
            const auto kitchenIt = std::find_if(p.roomLabels.begin(), p.roomLabels.end(),
                [](const std::string& r) { return r.find("Kitchen") != std::string::npos; });
            if (kitchenIt != p.roomLabels.end()) {
                p.lunchPending = false;
                ctx.pushEvent(std::make_shared<PersonLunchEvent>(
                    std::max(p.lunchTime, this->time), this->person, *kitchenIt));
                return;
            }
        }

        if (p.departureTime < nextExecutionTime) {
            auto elevatorIt = std::find_if(p.roomLabels.begin(), p.roomLabels.end(),
                [](const std::string& r) { return r.find("Elevator") != std::string::npos; });
            if (elevatorIt != p.roomLabels.end()) {
                ctx.setPersonLocation(p.firstName, *elevatorIt);
            }
            const int departAt = std::max(p.departureTime, this->time);
            ctx.pushEvent(std::make_shared<PersonDepartureEvent>(departAt, this->person));
        } else {
            ctx.pushEvent(std::make_shared<PersonTransitionEvent>(
                static_cast<int>(nextExecutionTime), this->person));
        }
    }

    std::string getName() const override {
        return std::format("{} arrived at {}", person->firstName, targetRoom);
    }
    des::EventType getType() const override { return des::EventType::PERSON_ROOM_ARRIVED; }
};

inline void PersonTransitionEvent::execute(ISimContext& ctx) {
    auto& p = *this->person;

    if (p.busy) {
        if (!targetRoom.empty()) {
            ctx.notifyEvent(*this);
        }
        ctx.pushEvent(this->withTime(busyRetryAt(ctx, this->time)));
        return;
    }

    const std::string currentRoom = ctx.getPersonLocation(p.firstName);
    targetRoom = currentRoom;

    if (currentRoom == "OUTDOOR") {
        ctx.notifyEvent(*this);
        return;
    }

    auto it = std::find(p.roomLabels.begin(), p.roomLabels.end(), currentRoom);

    // Person was moved outside their known rooms (e.g. by accompany) — return to workplace
    if (it == p.roomLabels.end()) {
        ctx.notifyEvent(*this);
        ctx.setPersonLocation(p.firstName, p.workplace);
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(
            this->time + rnd::uni(p.rng, 60, ONE_HOUR), this->person));
        return;
    }

    int currentIndex = std::distance(p.roomLabels.begin(), it);
    const std::vector<double>& row = p.transitionMatrix.at(currentIndex);
    int nextRoomIdx = rnd::discrete_dist(p.rng, row);

    const std::string nextRoom = p.roomLabels.at(nextRoomIdx);
    targetRoom = nextRoom;
    ctx.notifyEvent(*this);

    const double walkTime = personWalkTime(ctx, p, currentRoom, nextRoom);
    ctx.setPersonLocation(p.firstName, IN_TRANSIT);
    ctx.pushEvent(std::make_shared<PersonRoomArrivedEvent>(
        static_cast<int>(this->time + walkTime), this->person, nextRoom));
}
