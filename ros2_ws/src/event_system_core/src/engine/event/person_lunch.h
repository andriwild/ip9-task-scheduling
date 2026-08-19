#pragma once

#include <format>
#include <utility>

#include "engine/event/person_event.h"
#include "model/person.h"

namespace des {

class PersonLunchArrivedEvent final : public PersonEvent {
public:
    explicit PersonLunchArrivedEvent(const int time, Person* p, std::string room) :
        PersonEvent(time, p)
    {
        targetRoom = std::move(room);
    }

    [[nodiscard]] std::shared_ptr<IEvent> withTime(const int newTime) const override {
        auto copy = std::make_shared<PersonLunchArrivedEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& p = *this->person;

        if (p.busy) {
            ctx.notifyEvent(*this);
            scheduleTransition(ctx, busyRetryAt(ctx, this->time));
            return;
        }

        moveTo(ctx, targetRoom);
        ctx.notifyEvent(*this);
        scheduleTransition(ctx, static_cast<int>(this->time + p.lunchDuration));
    }

    [[nodiscard]] std::string getName() const override {
        return std::format("{} having lunch at {}", person->firstName, targetRoom);
    }
    [[nodiscard]] EventType getType() const override { return EventType::PERSON_ROOM_ARRIVED; }
};

class PersonLunchEvent final : public PersonEvent {
public:
    explicit PersonLunchEvent(const int time, Person* p, std::string room) :
        PersonEvent(time, p)
    {
        targetRoom = std::move(room);
    }

    [[nodiscard]] std::shared_ptr<IEvent> withTime(const int newTime) const override {
        auto copy = std::make_shared<PersonLunchEvent>(*this);
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
        ctx.notifyEvent(*this);

        const double walkTime = personWalkTime(ctx, p, currentRoom, targetRoom);
        moveTo(ctx, IN_TRANSIT);
        ctx.pushEvent(std::make_shared<PersonLunchArrivedEvent>(
            static_cast<int>(this->time + walkTime), this->person, targetRoom));
    }

    [[nodiscard]] EventType getType() const override { return EventType::PERSON_TRANSITION; }
};

}  // namespace des
