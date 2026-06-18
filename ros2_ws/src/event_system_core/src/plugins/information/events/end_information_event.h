#pragma once

#include "model/event/base.h"
#include "model/event/mission_complete_event.h"
#include "model/i_sim_context.h"
#include "plugins/i_order.h"

class EndInformationEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit EndInformationEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<EndInformationEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        m_order->state = des::MissionState::COMPLETED;
        ctx.notifyEvent(*this);
        ctx.popInterrupt(m_order);
        ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, m_order));
        ctx.tickBT();
    }

    std::string getName() const override { return "End Information"; }
    des::EventType getType() const override { return des::EventType::INFORMATION; }
};
