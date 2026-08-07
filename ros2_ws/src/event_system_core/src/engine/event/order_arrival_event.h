#pragma once

#include <format>
#include <rclcpp/rclcpp.hpp>

#include "engine/contracts/i_event.h"
#include "engine/event/mission_complete_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/order.h"
#include "util/types.h"

namespace des {

class OrderArrivalEvent final : public IEvent {
    OrderPtr m_order;
public:
    explicit OrderArrivalEvent(int time, OrderPtr order)
        : IEvent(time), m_order(std::move(order)) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<OrderArrivalEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.publishMission(m_order, time);
        switch (m_order->execution) {
            case ExecutionMode::BACKGROUND:
            case ExecutionMode::SCHEDULED:
                ctx.addScheduledMission(m_order);
                break;
            case ExecutionMode::INTERRUPT:
                if (!ctx.pushInterrupt(m_order)) {
                    m_order->state = MissionState::REJECTED;
                    ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, m_order));
                }
                break;
        }
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Order Arrival ({})", m_order->type);
    }
    EventType getType() const override { return EventType::ORDER_ARRIVAL; }
    int getMissionId() const override { return m_order ? m_order->id : -1; }
    OrderPtr getOrder() const override {
        return m_order;
    }
};

}  // namespace des
