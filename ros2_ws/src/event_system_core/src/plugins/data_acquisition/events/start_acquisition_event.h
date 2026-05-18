#pragma once

#include "model/event/base.h"
#include "model/i_sim_context.h"
#include "plugins/i_order.h"
#include "execute_acquisition_event.h"

class StartAcquisitionEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit StartAcquisitionEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    void execute(ISimContext& ctx) override {
        m_order->state = des::MissionState::IN_PROGRESS;
        ctx.notifyEvent(*this);
        const int duration = static_cast<int>(ctx.getConfig()->dataAcquisitionDuration);
        ctx.pushEvent(std::make_shared<EndAcquisitionEvent>(this->time + duration, m_order));
    }

    std::string getName() const override { return "Start Acquisition"; }
    des::EventType getType() const override { return des::EventType::DATA_ACQUISITION_START; }
};
