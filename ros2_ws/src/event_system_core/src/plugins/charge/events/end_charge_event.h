#pragma once

#include "engine/contracts/i_event.h"
#include "engine/event/mission_complete_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/order.h"

class EndChargeEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit EndChargeEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<EndChargeEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->completeCharge();
        m_order->state = des::MissionState::COMPLETED;
        ctx.notifyEvent(*this);
        ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, m_order));
        ctx.tickBT();
    }

    std::string getName() const override { return "End Charge"; }
    des::EventType getType() const override { return des::EventType::CHARGE_MISSION; }
};
