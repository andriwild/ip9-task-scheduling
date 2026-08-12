#pragma once

#include <cassert>
#include <format>

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot_state.h"
#include "plugins/order_registry.h"

namespace des {

class MissionStartEvent final : public IEvent {
public:
    OrderPtr orderPtr;
    explicit MissionStartEvent(const int time, const OrderPtr& order)
        : IEvent(time)
        , orderPtr(order)
    {}

    [[nodiscard]] std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<MissionStartEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& plugin = OrderRegistry::instance().get(orderPtr->type);
        plugin.onMissionStart(ctx, *orderPtr);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    [[nodiscard]] std::string getName() const override {
        return std::format("Mission {} Start", orderPtr->id);
    }
    [[nodiscard]] EventType getType() const override { return EventType::MISSION_START; }
    [[nodiscard]] int getMissionId() const override { return orderPtr ? orderPtr->id : -1; }
};

}  // namespace des
