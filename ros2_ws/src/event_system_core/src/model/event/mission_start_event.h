#pragma once

#include <cassert>
#include <format>

#include "base.h"
#include "../i_sim_context.h"
#include "../robot_state.h"

class MissionStartEvent final : public IEvent {
public:
    std::shared_ptr<des::Appointment> appointment;
    explicit MissionStartEvent(const int time, const std::shared_ptr<des::Appointment> &appointment)
        : IEvent(time)
        , appointment(appointment)
    {}

    void execute(ISimContext& ctx) override {
        const std::string person = appointment->personName;
        assert(ctx.hasEmployee(person));
        const auto& locations = ctx.getPersonByName(person)->roomLabels;
        assert(!locations.empty());
        ctx.changeRobotState(std::make_unique<SearchState>(locations));
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Mission {} Start", appointment->id);
    }
    des::EventType getType() const override { return des::EventType::MISSION_START; }
};
