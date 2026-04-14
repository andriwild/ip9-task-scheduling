#pragma once

#include <format>

#include "base.h"
#include "../i_sim_context.h"

class MissionDispatchEvent final : public IEvent {
public:
    std::shared_ptr<des::Appointment> appointment;
    explicit MissionDispatchEvent(const int time, const std::shared_ptr<des::Appointment> &appointment)
        : IEvent(time)
        , appointment(appointment)
    {}

    void execute(ISimContext& ctx) override {
        ctx.addPendingMission(this->appointment);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Mission {} Dispatch: {}", appointment->id, appointment->personName);
    }
    des::EventType getType() const override { return des::EventType::MISSION_DISPATCH; }
};
