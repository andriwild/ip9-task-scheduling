#pragma once

#include "base.h"
#include "start_drive_event.h"
#include "../i_sim_context.h"
#include "../robot_state.h"
#include "../../plugins/accompany/accompany_order.h"

class StartAccompanyEvent final : public IEvent {
public:
    explicit StartAccompanyEvent(const int time) : IEvent(time) {}

    void execute(ISimContext& ctx) override {
        const auto& accompany = static_cast<const AccompanyOrder&>(*ctx.getOrderPtr());
        const auto& personName = accompany.personName;
        if (ctx.hasEmployee(personName)) {
            ctx.getPersonByName(personName)->busy = true;
        }
        ctx.changeRobotState(std::make_unique<AccompanyState>());
        ctx.pushEvent(std::make_shared<StartDriveEvent>(time, accompany.roomName));
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Start Accompany"; }
    des::EventType getType() const override { return des::EventType::START_ACCOMPANY; }
};
