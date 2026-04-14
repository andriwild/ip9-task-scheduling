#pragma once

#include <algorithm>
#include <format>
#include <iterator>
#include <utility>

#include "base.h"
#include "../i_sim_context.h"
#include "../../util/rnd.h"

class PersonDepartureEvent;

class PersonTransitionEvent : public IEvent {
public:
    const std::shared_ptr<des::Person> person;
    std::string targetRoom;
    explicit PersonTransitionEvent(const int time, std::shared_ptr<des::Person> p) :
        IEvent(time),
        person(std::move(p))
    {}

    void execute(ISimContext& ctx) override;

    std::string getName() const override {
        return std::format("{} moved to {}", person->firstName, targetRoom);
    }
    des::EventType getType() const override { return des::EventType::PERSON_TRANSITION; }
    std::string getColor() const override { return person->color; }
};

class PersonArrivedEvent final : public PersonTransitionEvent {
public:
    explicit PersonArrivedEvent(const int time, std::shared_ptr<des::Person> p) :
        PersonTransitionEvent(time, std::move(p))
    {}

    void execute(ISimContext& ctx) override {
        auto& p = *this->person;

        if (p.busy) {
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
                this->time + rnd::uni(ctx.rng(), 60, ONE_HOUR), this->person));
            return;
        }

        int currentIndex = std::distance(p.roomLabels.begin(), it);
        const std::vector<double>& row = p.transitionMatrix.at(currentIndex);
        int nextRoomIdx = rnd::discrete_dist(ctx.rng(), row);

        ctx.setPersonLocation(p.firstName, p.roomLabels.at(nextRoomIdx));
        double nextExecutionTime = this->time + rnd::uni(ctx.rng(), 10, 30);
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(nextExecutionTime, this->person));
    }

    std::string getName() const override {
        return std::format("{} arrived to {}", person->firstName, targetRoom);
    }
    des::EventType getType() const override { return des::EventType::PERSON_ARRIVED; }
};

class PersonDepartureEvent final : public PersonTransitionEvent {
public:
    explicit PersonDepartureEvent(const int time, std::shared_ptr<des::Person> p) :
        PersonTransitionEvent(time, std::move(p))
    {}

    void execute(ISimContext& ctx) override {
        if (this->person->busy) {
            return;
        }
        targetRoom = "OUTDOOR";
        ctx.notifyEvent(*this);
        ctx.setPersonLocation(this->person->firstName, "OUTDOOR");
    }

    std::string getName() const override {
        return std::format("{} leaved to {}", person->firstName, targetRoom);
    }
    des::EventType getType() const override { return des::EventType::PERSON_DEPARTURE; }
};

inline void PersonTransitionEvent::execute(ISimContext& ctx) {
    auto& p = *this->person;

    if (p.busy) {
        if (!targetRoom.empty()) {
            ctx.notifyEvent(*this);
        }
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
            this->time + rnd::uni(ctx.rng(), 60, ONE_HOUR), this->person));
        return;
    }

    int currentIndex = std::distance(p.roomLabels.begin(), it);
    const std::vector<double>& row = p.transitionMatrix.at(currentIndex);
    int nextRoomIdx = rnd::discrete_dist(ctx.rng(), row);

    const std::string nextRoom = p.roomLabels.at(nextRoomIdx);
    targetRoom = nextRoom;
    ctx.notifyEvent(*this);
    ctx.setPersonLocation(p.firstName, nextRoom);

    double nextExecutionTime = this->time + p.getStayDuration(nextRoom, ctx.rng());

    if (p.departureTime < nextExecutionTime) {
        auto elevatorIt = std::find_if(p.roomLabels.begin(), p.roomLabels.end(),
            [](const std::string& r) { return r.find("Elevator") != std::string::npos; });
        if (elevatorIt != p.roomLabels.end()) {
            ctx.setPersonLocation(p.firstName, *elevatorIt);
        }
        ctx.pushEvent(std::make_shared<PersonDepartureEvent>(p.departureTime, this->person));
    } else {
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(nextExecutionTime, this->person));
    }
}
