#pragma once

#include <cassert>
#include <format>

#include "base.h"
#include "../i_sim_context.h"
#include "../robot_state.h"
#include "../../plugins/accompany/accompany_order.h"

class MissionStartEvent final : public IEvent {
public:
    des::OrderPtr orderPtr;
    explicit MissionStartEvent(const int time, const des::OrderPtr& order)
        : IEvent(time)
        , orderPtr(order)
    {}

    void execute(ISimContext& ctx) override {
        const auto& accompany = static_cast<const AccompanyOrder&>(*orderPtr);
        const std::string& person = accompany.personName;
        assert(ctx.hasEmployee(person));
        const auto& locations = ctx.getPersonByName(person)->roomLabels;
        assert(!locations.empty());
        ctx.changeRobotState(std::make_unique<SearchState>(locations));
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Mission {} Start", orderPtr->id);
    }
    des::EventType getType() const override { return des::EventType::MISSION_START; }
};
