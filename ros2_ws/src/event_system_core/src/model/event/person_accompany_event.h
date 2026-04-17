#pragma once

#include <format>
#include <memory>
#include <string>
#include <utility>

#include "base.h"
#include "../i_sim_context.h"
#include "../person.h"

class PersonAccompanyDepartureEvent final : public IEvent {
public:
    const std::shared_ptr<des::Person> person;
    const std::string currentRoom;

    PersonAccompanyDepartureEvent(const int time,
                                  std::shared_ptr<des::Person> p,
                                  std::string currentRoom)
        : IEvent(time)
        , person(std::move(p))
        , currentRoom(std::move(currentRoom))
    {}

    void execute(ISimContext& ctx) override {
        ctx.notifyEvent(*this);
    }

    std::string getName() const override {
        return std::format("{} moved to {}", person->firstName, currentRoom);
    }
    des::EventType getType() const override { return des::EventType::PERSON_ACCOMPANY_DEPARTURE; }
    std::string getColor() const override { return person->color; }
};

class PersonAccompanyArrivedEvent final : public IEvent {
public:
    const std::shared_ptr<des::Person> person;
    const std::string arrivalRoom;

    PersonAccompanyArrivedEvent(const int time,
                                std::shared_ptr<des::Person> p,
                                std::string arrivalRoom)
        : IEvent(time)
        , person(std::move(p))
        , arrivalRoom(std::move(arrivalRoom))
    {}

    void execute(ISimContext& ctx) override {
        ctx.notifyEvent(*this);
    }

    std::string getName() const override {
        return std::format("{} arrived to {}", person->firstName, arrivalRoom);
    }
    des::EventType getType() const override { return des::EventType::PERSON_ACCOMPANY_ARRIVED; }
    std::string getColor() const override { return person->color; }
};
