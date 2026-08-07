#pragma once

#include <format>
#include <utility>

#include "engine/contracts/i_event.h"
#include "engine/event/person_transition.h"
#include "engine/contracts/i_sim_context.h"

namespace des {

class AppointmentEndEvent final : public IEvent {
public:
    Person* person;
    explicit AppointmentEndEvent(const int time, Person* p)
        : IEvent(time), person(std::move(p)) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<AppointmentEndEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        person->busy = false;
        ctx.notifyEvent(*this);
    }

    std::string getName() const override {
        return std::format("{} Appointment End", person->firstName);
    }
    EventType getType() const override { return EventType::APPOINTMENT_END; }
};

}  // namespace des


