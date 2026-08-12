#pragma once

#include "engine/contracts/i_event.h"
#include "engine/event/mission_complete_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/order.h"

namespace des {

class EndAcquisitionEvent final : public IEvent {
    OrderPtr m_order;
public:
    explicit EndAcquisitionEvent(const int time, const OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<EndAcquisitionEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        m_order->state = OrderState::COMPLETED;
        ctx.notifyEvent(*this);
        ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, m_order));
        ctx.tickBT();
    }

    std::string getName() const override { return "End Acquisition"; }
    EventType getType() const override { return EventType::DATA_ACQUISITION; }
};

}  // namespace des
