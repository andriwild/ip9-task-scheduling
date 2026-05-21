#pragma once

#include <cassert>
#include <cmath>

#include "model/event/base.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "plugins/i_order.h"
#include "plugins/clean/clean_plugin.h"
#include "end_clean_event.h"

class StartCleanEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit StartCleanEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    void execute(ISimContext& ctx) override {
        m_order->state = des::MissionState::IN_PROGRESS;
        ctx.notifyEvent(*this);

        const std::string robotLocation = ctx.getRobot()->getLocation();
        const double roomArea           = ctx.getSearchArea(robotLocation);
        const double broomFootprint     = cleanConfig().cleaningArea;
        assert(broomFootprint > 0.0);
        const double broomSide  = std::sqrt(broomFootprint);
        const double steps      = (roomArea / broomFootprint) + 1;
        const int cleanTime     = static_cast<int>(steps * (2.0 * broomSide / ctx.getConfig()->robotSpeed));
        assert(cleanTime > 0);

        ctx.pushEvent(std::make_shared<EndCleanEvent>(this->time + cleanTime, m_order));
    }

    std::string getName() const override { return "Start Clean"; }
    des::EventType getType() const override { return des::EventType::CLEAN_START; }
};
