#pragma once

#include "model/event/base.h"
#include "model/i_sim_context.h"
#include "end_information_event.h"

class StartInformationEvent final : public IEvent {
public:
    explicit StartInformationEvent(const int time) : IEvent(time) {}

    void execute(ISimContext& ctx) override {
        ctx.notifyEvent(*this);
        const int duration = static_cast<int>(ctx.getConfig()->informationDuration);
        ctx.pushEvent(std::make_shared<EndInformationEvent>(this->time + duration));
    }

    std::string getName() const override { return "Start Information"; }
    des::EventType getType() const override { return des::EventType::INFORMATION_START; }
};
