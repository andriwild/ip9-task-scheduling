#pragma once

#include <format>

#include "base.h"
#include "appointment_end_event.h"
#include "../i_sim_context.h"
#include "../../plugins/accompany/accompany_order.h"

class MissionCompleteEvent final : public IEvent {
public:
    des::OrderPtr orderPtr;
    explicit MissionCompleteEvent(const int time, const des::OrderPtr& order)
        : IEvent(time)
        , orderPtr(order)
    {}

    void execute(ISimContext& ctx) override {
        if (auto accompany = std::dynamic_pointer_cast<AccompanyOrder>(orderPtr)) {
            const auto& personName = accompany->personName;
            if (ctx.hasEmployee(personName)) {
                auto person = ctx.getPersonByName(personName);
                int endTime = this->time + static_cast<int>(ctx.getConfig()->appointmentDuration);
                ctx.pushEvent(std::make_shared<AppointmentEndEvent>(endTime, person));
            }
        }
        ctx.completeOrder(this->orderPtr);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Mission {} Complete: {}", orderPtr->id, des::missionStateStr(orderPtr->state));
    }
    des::EventType getType() const override { return des::EventType::MISSION_COMPLETE; }
};
