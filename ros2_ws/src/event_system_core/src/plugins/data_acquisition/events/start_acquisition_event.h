#pragma once

#include "model/event/base.h"
#include "model/i_sim_context.h"
#include "plugins/i_order.h"
#include "plugins/data_acquisition/data_acquisition_plugin.h"
#include "execute_acquisition_event.h"

class StartAcquisitionEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit StartAcquisitionEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartAcquisitionEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        m_order->state = des::MissionState::IN_PROGRESS;
        ctx.notifyEvent(*this);
        const int duration = static_cast<int>(dataAcquisitionConfig().dataAcquisitionDuration);
        ctx.startActivity(std::make_shared<EndAcquisitionEvent>(this->time + duration, m_order));
    }

    std::string getName() const override { return "Start Acquisition"; }
    des::EventType getType() const override { return des::EventType::DATA_ACQUISITION_START; }
};
