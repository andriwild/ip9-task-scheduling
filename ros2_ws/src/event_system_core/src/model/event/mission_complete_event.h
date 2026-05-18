#pragma once

#include <format>

#include "base.h"
#include "../../plugins/order_registry.h"
#include "../i_sim_context.h"
#include "util/types.h"

class MissionCompleteEvent final : public IEvent {
public:
    des::OrderPtr orderPtr;
    explicit MissionCompleteEvent(const int time, const des::OrderPtr& order)
        : IEvent(time)
        , orderPtr(order)
    {}

    void execute(ISimContext& ctx) override {
        auto& plugin = OrderRegistry::instance().get(orderPtr->type);
        plugin.onMissionEnd(ctx, *orderPtr);
        // popInterrupt is done in EndXxxEvent for INTERRUPT plugins so the subsequent
        // tickBT sees the correct m_current.
        if (orderPtr->execution != des::ExecutionMode::INTERRUPT) {
            ctx.completeOrder(this->orderPtr);
        }
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Mission {} Complete: {}", orderPtr->id, des::missionStateStr(orderPtr->state));
    }
    des::EventType getType() const override { return des::EventType::MISSION_COMPLETE; }
};
