#pragma once

#include <cassert>
#include <format>

#include "base.h"
#include "../i_sim_context.h"
#include "../robot_state.h"
#include "../../plugins/order_registry.h"

class MissionStartEvent final : public IEvent {
public:
    des::OrderPtr orderPtr;
    explicit MissionStartEvent(const int time, const des::OrderPtr& order)
        : IEvent(time)
        , orderPtr(order)
    {}

    void execute(ISimContext& ctx) override {
        auto orderPtr = ctx.getOrderPtr();
        auto& plugin = OrderRegistry::instance().get(orderPtr->type);
        plugin.onMissionStart(ctx, *orderPtr);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Mission {} Start", orderPtr->id);
    }
    des::EventType getType() const override { return des::EventType::MISSION_START; }
};
