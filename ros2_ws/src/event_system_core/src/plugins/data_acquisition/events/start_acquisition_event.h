#pragma once

#include "model/event/base.h"
#include "model/i_sim_context.h"
#include "execute_acquisition_event.h"

class StartAcquisitionEvent final : public IEvent {
public:
    explicit StartAcquisitionEvent(const int time) : IEvent(time) {}

    void execute(ISimContext& ctx) override {
        ctx.notifyEvent(*this);
        const int duration = static_cast<int>(ctx.getConfig()->dataAcquisitionDuration);
        ctx.pushEvent(std::make_shared<EndAcquisitionEvent>(this->time + duration));
    }

    std::string getName() const override { return "Start Acquisition"; }
    des::EventType getType() const override { return des::EventType::DATA_ACQUISITION_START; }
};
