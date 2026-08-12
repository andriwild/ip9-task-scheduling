#pragma once

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/order.h"
#include "plugins/information/information_plugin.h"
#include "plugins/information/information_order.h"
#include "util/rnd.h"
#include "end_information_event.h"

namespace des {

class StartInformationEvent final : public IEvent {
    OrderPtr m_order;
public:
    explicit StartInformationEvent(const int time, const OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartInformationEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        m_order->state = OrderState::IN_PROGRESS;
        ctx.notifyEvent(*this);
        double sampled = static_cast<const InformationOrder&>(*m_order).sampledDuration;
        if (sampled < 0.0) {
            const auto& cfg = informationConfig();
            sampled = rnd::uni(ctx.robotRng(), cfg.informationDurationMin, cfg.informationDurationMax);
        }
        const int duration = static_cast<int>(sampled < 1.0 ? 1.0 : sampled);
        ctx.pushEvent(std::make_shared<EndInformationEvent>(this->time + duration, m_order));
    }

    std::string getName() const override { return "Start Information"; }
    EventType getType() const override { return EventType::INFORMATION_START; }
};

}  // namespace des
