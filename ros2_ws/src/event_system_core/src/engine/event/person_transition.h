/*
 * Movement events of the simulated people: arrival, room transitions
 * from the transition matrix, lunch and departure, which reseeds the
 * occupancy for the next day.
 * Persons draw from their own RNG stream, busy persons defer their
 * events until the robot releases them (accompany).
 *
 */
#pragma once

#include <algorithm>
#include <format>
#include <iterator>
#include <utility>

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/occupancy.h"
#include "util/rnd.h"
#include "util/constants.h"

namespace des {

class PersonDepartureEvent;

inline int busyRetryAt(ISimContext& ctx, const int now) {
    return now + static_cast<int>(rnd::uni(ctx.robotRng(), 60, 300));
}

inline double personWalkTime(ISimContext& ctx, Person& person, const std::string& from, const std::string& to) {
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
    Person* const person;
    std::string targetRoom;

    // Set on every move, read by the move trace.
    // OUTDOOR and IN_TRANSIT are no rooms, so they have a name but no position.
    std::string m_room;
    std::optional<Point> m_at;
    explicit PersonTransitionEvent(const int time, Person* p) :
        IEvent(time),
        person(std::move(p))
    {}

    void moveTo(ISimContext& ctx, const std::string& room) {
        ctx.setPersonLocation(person->firstName, room);
        m_room = room;
        m_at = ctx.getPersonPosition(person->firstName);
    }


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
    EventType getType() const override { return EventType::PERSON_TRANSITION; }
    std::string getColor() const override { return person->color; }
    int getMissionId() const override { return -1; }

};

class PersonArrivedEvent final : public PersonTransitionEvent {
public:
    explicit PersonArrivedEvent(const int time, Person* p) :
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
            moveTo(ctx, p.workplace);
            ctx.pushEvent(std::make_shared<PersonTransitionEvent>(
                this->time + rnd::uni(p.rng, 60, ONE_HOUR), this->person));
            return;
        }

        int currentIndex = std::distance(p.roomLabels.begin(), it);
        const std::vector<double>& row = p.transitionMatrix.at(currentIndex);
        int nextRoomIdx = rnd::discrete_dist(p.rng, row);

        moveTo(ctx, p.roomLabels.at(nextRoomIdx));
        double nextExecutionTime = this->time + rnd::uni(p.rng, 10, 30);
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(nextExecutionTime, this->person));
    }

    std::string getName() const override {
        return std::format("{} arrived to {}", person->firstName, targetRoom);
    }
    EventType getType() const override { return EventType::PERSON_ARRIVED; }
};

class PersonDepartureEvent final : public PersonTransitionEvent {
public:
    explicit PersonDepartureEvent(const int time, Person* p) :
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
        moveTo(ctx, "OUTDOOR");
        ctx.notifyEvent(*this);

        const int nextDayBase = (this->time / SECONDS_PER_DAY + 1) * SECONDS_PER_DAY;
        sampleOccupancy(*ctx.getConfig(), this->person->rng, nextDayBase, *this->person);
        const auto simEnd = ctx.getSimulationEndTime();
        if (simEnd.has_value() && this->person->arrivalTime < simEnd.value()) {
            ctx.pushEvent(std::make_shared<PersonArrivedEvent>(this->person->arrivalTime, this->person));
        }
    }

    std::string getName() const override {
        return std::format("{} leaved to {}", person->firstName, targetRoom);
    }
    EventType getType() const override { return EventType::PERSON_DEPARTURE; }
};

class PersonLunchArrivedEvent final : public PersonTransitionEvent {
public:
    explicit PersonLunchArrivedEvent(const int time, Person* p, std::string room) :
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

        moveTo(ctx, targetRoom);
        ctx.notifyEvent(*this);
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(
            static_cast<int>(this->time + p.lunchDuration), this->person));
    }

    std::string getName() const override {
        return std::format("{} having lunch at {}", person->firstName, targetRoom);
    }
    EventType getType() const override { return EventType::PERSON_ROOM_ARRIVED; }
};

class PersonLunchEvent final : public PersonTransitionEvent {
public:
    explicit PersonLunchEvent(const int time, Person* p, std::string room) :
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
        moveTo(ctx, IN_TRANSIT);
        ctx.pushEvent(std::make_shared<PersonLunchArrivedEvent>(
            static_cast<int>(this->time + walkTime), this->person, targetRoom));
    }

    EventType getType() const override { return EventType::PERSON_TRANSITION; }
};

class PersonRoomArrivedEvent final : public PersonTransitionEvent {
public:
    explicit PersonRoomArrivedEvent(const int time, Person* p, std::string room) :
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

        moveTo(ctx, targetRoom);
        ctx.notifyEvent(*this);

        double nextExecutionTime = this->time + p.getStayDuration(ctx.room(targetRoom).m_roomType, p.rng);

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
                moveTo(ctx, *elevatorIt);
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
    EventType getType() const override { return EventType::PERSON_ROOM_ARRIVED; }
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
        moveTo(ctx, p.workplace);
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
    moveTo(ctx, IN_TRANSIT);
    ctx.pushEvent(std::make_shared<PersonRoomArrivedEvent>(
        static_cast<int>(this->time + walkTime), this->person, nextRoom));
}

}  // namespace des
