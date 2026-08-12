#pragma once

#include "engine/contracts/i_event.h"
#include "engine/event/mission_complete_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/order.h"

namespace des {

class EndInformationEvent final : public IEvent {
    OrderPtr m_order;
public:
    explicit EndInformationEvent(const int time, const OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<EndInformationEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        m_order->state = OrderState::COMPLETED;
        ctx.notifyEvent(*this);
        ctx.popInterrupt(m_order);
        ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, m_order));
        ctx.tickBT();
    }

    std::string getName() const override { return "End Information"; }
    EventType getType() const override { return EventType::INFORMATION; }
    int priority() const override { return 1; }
};

}  // namespace des
