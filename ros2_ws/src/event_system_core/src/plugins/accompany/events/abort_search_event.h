#pragma once

#include "model/event/base.h"
#include "model/event/mission_complete_event.h"
#include "model/i_sim_context.h"
#include "model/robot_state.h"
#include "plugins/i_order.h"

class AbortSearchEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit AbortSearchEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    void execute(ISimContext& ctx) override {
        m_order->state = des::MissionState::FAILED;
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, m_order));
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Abort Search"; }
    des::EventType getType() const override { return des::EventType::ABORT_SEARCH; }
};
