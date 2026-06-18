#pragma once

#include "model/event/base.h"
#include "person_accompany_event.h"
#include "model/event/start_drive_event.h"
#include "model/i_sim_context.h"
#include "model/robot_state.h"
#include "plugins/accompany/accompany_order.h"
#include "plugins/accompany/states.h"

class StartAccompanyEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit StartAccompanyEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartAccompanyEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        const auto& accompany = static_cast<const AccompanyOrder&>(*m_order);
        const auto& personName = accompany.personName;
        if (ctx.hasEmployee(personName)) {
            auto person = ctx.getPersonByName(personName);
            const std::string currentRoom = ctx.getPersonLocation(personName);
            ctx.pushEvent(std::make_shared<PersonAccompanyDepartureEvent>(time, person, currentRoom));
        }
        ctx.changeRobotState(std::make_unique<AccompanyState>());
        requestDrive(ctx, accompany.roomName);
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Start Accompany"; }
    des::EventType getType() const override { return des::EventType::START_ACCOMPANY; }
};
