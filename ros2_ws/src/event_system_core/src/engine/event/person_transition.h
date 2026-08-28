/*
 * Movement events of the simulated people: arrival, room transitions
 * from the transition matrix and departure, which reseeds the
 * occupancy for the next day. Lunch is in person_lunch.h.
 * Persons draw from their own RNG stream, busy persons defer their
 * events until the robot releases them (accompany).
 *
 */
#pragma once

#include <algorithm>
#include <format>
#include <utility>
#include <vector>

#include "engine/event/person_event.h"
#include "engine/event/person_lunch.h"
#include "model/occupancy.h"
#include "util/constants.h"
#include "model/person.h"
#include "model/room.h"

namespace des {

inline std::string accessRoomFor(const ISimContext& ctx, const Person& p) {
    const auto isAccess = [&ctx](const std::string& r) {
        return ctx.room(r).m_roomType == RoomType::ACCESS;
    };
    const auto own = std::ranges::find_if(p.roomLabels, isAccess);
    if (own != p.roomLabels.end()) {
        return *own;
    }
    const std::vector<std::string> names = ctx.roomNames();
    const auto any = std::ranges::find_if(names, isAccess);
    return any == names.end() ? std::string{} : *any;
}

class PersonArrivedEvent final : public PersonEvent {
public:
    explicit PersonArrivedEvent(const int time, Person* p) :
        PersonEvent(time, p)
    {}

    [[nodiscard]] std::shared_ptr<IEvent> withTime(const int newTime) const override {
        auto copy = std::make_shared<PersonArrivedEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& p = *this->person;

        if (p.busy) {
            retryLater(ctx);
            return;
        }

        const std::string currentRoom = ctx.getPersonLocation(p.firstName);
        targetRoom = currentRoom;
        ctx.notifyEvent(*this);

        const std::optional<std::string> nextRoom = drawNextRoom(p, currentRoom);

        // Person arrived at unknown room (e.g. after accompany) — return to workplace
        if (!nextRoom.has_value()) {
            moveTo(ctx, p.workplace);
            scheduleTransition(ctx, this->time + static_cast<int>(rnd::uni(p.rng, 60, ONE_HOUR)));
            return;
        }

        // Arrival at the building, person is out of his transition matrix
        moveTo(ctx, nextRoom.value());
        scheduleTransition(ctx, this->time + static_cast<int>(rnd::uni(p.rng, 10, 30)));
    }

    [[nodiscard]] std::string getName() const override {
        return std::format("{} arrived to {}", person->firstName, targetRoom);
    }
    [[nodiscard]] EventType getType() const override { return EventType::PERSON_ARRIVED; }
};

class PersonDepartureEvent final : public PersonEvent {
public:
    explicit PersonDepartureEvent(const int time, Person* p) :
        PersonEvent(time, p)
    {}

    [[nodiscard]] std::shared_ptr<IEvent> withTime(const int newTime) const override {
        auto copy = std::make_shared<PersonDepartureEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        // accompany guard
        if (this->person->busy) {
            retryLater(ctx);
            return;
        }
        targetRoom = OUTDOOR;
        moveTo(ctx, OUTDOOR);
        ctx.notifyEvent(*this);

        const int nextDayBase = (this->time / SECONDS_PER_DAY + 1) * SECONDS_PER_DAY;
        sampleOccupancy(*ctx.getConfig(), this->person->rng, nextDayBase, *this->person);
        const auto simEnd = ctx.getSimulationEndTime();
        if (simEnd.has_value() && this->person->arrivalTime < simEnd.value()) {
            ctx.pushEvent(std::make_shared<PersonArrivedEvent>(this->person->arrivalTime, this->person));
        }
    }

    [[nodiscard]] std::string getName() const override {
        return std::format("{} leaved to {}", person->firstName, targetRoom);
    }
    [[nodiscard]] EventType getType() const override { return EventType::PERSON_DEPARTURE; }
};

class PersonAtExitEvent final : public PersonEvent {
public:
    explicit PersonAtExitEvent(const int time, Person* p, std::string room) :
        PersonEvent(time, p)
    {
        targetRoom = std::move(room);
    }

    [[nodiscard]] std::shared_ptr<IEvent> withTime(const int newTime) const override {
        auto copy = std::make_shared<PersonAtExitEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& p = *this->person;

        // accompany guard
        if (p.busy) {
            retryLater(ctx);
            return;
        }

        moveTo(ctx, targetRoom);
        ctx.notifyEvent(*this);

        const int departAt = std::max(p.departureTime, this->time);
        ctx.pushEvent(std::make_shared<PersonDepartureEvent>(departAt, this->person));
    }

    [[nodiscard]] std::string getName() const override {
        return std::format("{} waiting at {}", person->firstName, targetRoom);
    }
    [[nodiscard]] EventType getType() const override { return EventType::PERSON_ROOM_ARRIVED; }
};

class PersonLeaveEvent final : public PersonEvent {
    int m_walkTime;

public:
    explicit PersonLeaveEvent(const int time, Person* p, std::string room, const int walkTime) :
        PersonEvent(time, p),
        m_walkTime(walkTime)
    {
        targetRoom = std::move(room);
    }

    [[nodiscard]] std::shared_ptr<IEvent> withTime(const int newTime) const override {
        auto copy = std::make_shared<PersonLeaveEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        // accompany guard
        if (this->person->busy) {
            retryLater(ctx);
            return;
        }

        ctx.notifyEvent(*this);
        moveTo(ctx, IN_TRANSIT);
        ctx.pushEvent(std::make_shared<PersonAtExitEvent>(
            this->time + m_walkTime, this->person, targetRoom));
    }

    [[nodiscard]] EventType getType() const override { return EventType::PERSON_TRANSITION; }
};

class PersonRoomArrivedEvent final : public PersonEvent {
public:
    explicit PersonRoomArrivedEvent(const int time, Person* p, std::string room) :
        PersonEvent(time, p)
    {
        targetRoom = std::move(room);
    }

    [[nodiscard]] std::shared_ptr<IEvent> withTime(const int newTime) const override {
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
            scheduleTransition(ctx, busyRetryAt(ctx, this->time));
            return;
        }

        moveTo(ctx, targetRoom);
        ctx.notifyEvent(*this);

        double nextExecutionTime = this->time + p.getStayDuration(ctx.room(targetRoom).m_roomType, p.rng);

        if (p.lunchPending && p.lunchTime < nextExecutionTime && p.lunchTime < p.departureTime) {
            const auto kitchenIt = std::ranges::find_if(p.roomLabels,
                [&ctx](const std::string& r) { return ctx.room(r).m_roomType == RoomType::KITCHEN; });
            if (kitchenIt != p.roomLabels.end()) {
                p.lunchPending = false;
                ctx.pushEvent(std::make_shared<PersonLunchEvent>(
                    std::max(p.lunchTime, this->time), this->person, *kitchenIt));
                return;
            }
        }

        if (p.departureTime < nextExecutionTime) {
            const std::string accessRoom = accessRoomFor(ctx, p);
            if (accessRoom.empty()) {
                ctx.pushEvent(std::make_shared<PersonDepartureEvent>(
                    std::max(p.departureTime, this->time), this->person));
                return;
            }
            const double walkTime = personWalkTime(ctx, p, targetRoom, accessRoom);
            const double accessStay = p.getStayDuration(RoomType::ACCESS, p.rng);
            const int leaveAt = std::max(this->time, static_cast<int>(p.departureTime - walkTime - accessStay));
            ctx.pushEvent(std::make_shared<PersonLeaveEvent>(
                leaveAt, this->person, accessRoom, static_cast<int>(walkTime)));
        } else {
            scheduleTransition(ctx, static_cast<int>(nextExecutionTime));
        }
    }

    [[nodiscard]] std::string getName() const override {
        return std::format("{} arrived at {}", person->firstName, targetRoom);
    }
    [[nodiscard]] EventType getType() const override { return EventType::PERSON_ROOM_ARRIVED; }
};

class PersonTransitionEvent final : public PersonEvent {
public:
    explicit PersonTransitionEvent(const int time, Person* p) :
        PersonEvent(time, p)
    {}

    [[nodiscard]] std::shared_ptr<IEvent> withTime(const int newTime) const override {
        auto copy = std::make_shared<PersonTransitionEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& p = *this->person;

        if (p.busy) {
            if (!targetRoom.empty()) {
                ctx.notifyEvent(*this);
            }
            retryLater(ctx);
            return;
        }

        const std::string currentRoom = ctx.getPersonLocation(p.firstName);
        targetRoom = currentRoom;

        if (currentRoom == OUTDOOR) {
            ctx.notifyEvent(*this);
            return;
        }

        const std::optional<std::string> nextRoom = drawNextRoom(p, currentRoom);

        // Person was moved outside their known rooms (e.g. by accompany) — return to workplace
        if (!nextRoom.has_value()) {
            ctx.notifyEvent(*this);
            moveTo(ctx, p.workplace);
            scheduleTransition(ctx, this->time + static_cast<int>(rnd::uni(p.rng, 60, ONE_HOUR)));
            return;
        }

        targetRoom = nextRoom.value();
        ctx.notifyEvent(*this);

        const double walkTime = personWalkTime(ctx, p, currentRoom, targetRoom);
        moveTo(ctx, IN_TRANSIT);
        ctx.pushEvent(std::make_shared<PersonRoomArrivedEvent>(
            static_cast<int>(this->time + walkTime), this->person, targetRoom));
    }

    [[nodiscard]] EventType getType() const override { return EventType::PERSON_TRANSITION; }
};

inline void PersonEvent::scheduleTransition(ISimContext& ctx, const int at) const {
    ctx.pushEvent(std::make_shared<PersonTransitionEvent>(at, person));
}

}  // namespace des
