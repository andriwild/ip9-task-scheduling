#pragma once

#include <format>

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/order.h"

class BackgroundReleaseEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit BackgroundReleaseEvent(int time, des::OrderPtr order)
        : IEvent(time), m_order(std::move(order)) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<BackgroundReleaseEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.addBackgroundMission(m_order);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Background Release ({})", m_order->type);
    }
    des::EventType getType() const override { return des::EventType::BACKGROUND_RELEASE; }
    int getMissionId() const override { return m_order ? m_order->id : -1; }
    des::OrderPtr getOrder() const override {
        return m_order;
    }
};
