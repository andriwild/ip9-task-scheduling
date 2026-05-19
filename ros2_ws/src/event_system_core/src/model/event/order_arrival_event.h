#pragma once

#include <format>
#include <rclcpp/rclcpp.hpp>

#include "base.h"
#include "../i_sim_context.h"
#include "../../plugins/i_order.h"
#include "../../util/types.h"

class OrderArrivalEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit OrderArrivalEvent(int time, des::OrderPtr order)
        : IEvent(time), m_order(std::move(order)) {}

    void execute(ISimContext& ctx) override {
        ctx.publishMission(m_order, time);
        switch (m_order->execution) {
            case des::ExecutionMode::BACKGROUND:
            case des::ExecutionMode::SCHEDULED:
                ctx.addPendingMission(m_order);
                break;
            case des::ExecutionMode::INTERRUPT:
                ctx.pushInterrupt(m_order);
                break;
        }
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Order Arrival ({})", m_order->type);
    }
    des::EventType getType() const override { return des::EventType::ORDER_ARRIVAL; }
    int getMissionId() const override { return m_order ? m_order->id : -1; }
};
