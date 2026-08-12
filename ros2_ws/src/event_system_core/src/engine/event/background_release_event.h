#pragma once

#include <format>

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/order.h"

namespace des {

class BackgroundReleaseEvent final : public IEvent {
    OrderPtr m_order;
public:
    explicit BackgroundReleaseEvent(int time, OrderPtr order)
        : IEvent(time), m_order(std::move(order)) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<BackgroundReleaseEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.addBackgroundOrder(m_order);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Background Release ({})", m_order->type);
    }
    EventType getType() const override { return EventType::BACKGROUND_RELEASE; }
    int getMissionId() const override { return m_order ? m_order->id : -1; }
    OrderPtr getOrder() const override {
        return m_order;
    }
};

}  // namespace des
