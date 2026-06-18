#pragma once

#include <format>
#include <utility>

#include "model/event/base.h"
#include "model/event/person_transition.h"
#include "model/i_sim_context.h"

class AppointmentEndEvent final : public IEvent {
public:
    std::shared_ptr<des::Person> person;
    explicit AppointmentEndEvent(const int time, std::shared_ptr<des::Person> p)
        : IEvent(time), person(std::move(p)) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<AppointmentEndEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        person->busy = false;
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(this->time, person));
        ctx.notifyEvent(*this);
    }

    std::string getName() const override {
        return std::format("{} Appointment End", person->firstName);
    }
    des::EventType getType() const override { return des::EventType::APPOINTMENT_END; }
};


