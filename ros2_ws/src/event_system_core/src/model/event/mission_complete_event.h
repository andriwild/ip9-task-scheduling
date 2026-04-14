#pragma once

#include <format>

#include "base.h"
#include "appointment_end_event.h"
#include "../i_sim_context.h"

class MissionCompleteEvent final : public IEvent {
    std::shared_ptr<des::Appointment> appointment;
public:
    explicit MissionCompleteEvent(const int time, const std::shared_ptr<des::Appointment> &appt)
        : IEvent(time)
        , appointment(appt)
    {}

    void execute(ISimContext& ctx) override {
        const auto& personName = this->appointment->personName;
        if (ctx.hasEmployee(personName)) {
            auto person = ctx.getPersonByName(personName);
            int endTime = this->time + static_cast<int>(ctx.getConfig()->appointmentDuration);
            ctx.pushEvent(std::make_shared<AppointmentEndEvent>(endTime, person));
        }
        ctx.completeAppointment(this->appointment);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Mission {} Complete: {}", appointment->id, des::missionStateStr(appointment->state));
    }
    des::EventType getType() const override { return des::EventType::MISSION_COMPLETE; }
};
