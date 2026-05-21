#pragma once

#include "model/event/base.h"
#include "model/i_sim_context.h"
#include "plugins/i_order.h"
#include "plugins/information/information_plugin.h"
#include "end_information_event.h"

class StartInformationEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit StartInformationEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    void execute(ISimContext& ctx) override {
        m_order->state = des::MissionState::IN_PROGRESS;
        ctx.notifyEvent(*this);
        const int duration = static_cast<int>(informationConfig().informationDuration);
        ctx.pushEvent(std::make_shared<EndInformationEvent>(this->time + duration, m_order));
    }

    std::string getName() const override { return "Start Information"; }
    des::EventType getType() const override { return des::EventType::INFORMATION_START; }
};
