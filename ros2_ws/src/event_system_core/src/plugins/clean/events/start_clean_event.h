#pragma once

#include <cassert>
#include <cmath>

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/order.h"
#include "util/log.h"
#include "plugins/clean/clean_order.h"
#include "plugins/clean/clean_plugin.h"
#include "end_clean_event.h"

namespace des {

class StartCleanEvent final : public IEvent {
    OrderPtr m_order;
public:
    explicit StartCleanEvent(const int time, const OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartCleanEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        m_order->state = OrderState::IN_PROGRESS;
        ctx.notifyEvent(*this);

        const std::string& roomName = static_cast<const CleanOrder&>(*m_order).roomName;
        if (!ctx.room(roomName).m_area.has_value()) {
            DES_LOG_WARN("des.plugin.clean", "Room '%s' has no area, cleaning mission %d falls back to 1m2", roomName.c_str(), m_order->id);
        }
        static_cast<CleanOrder&>(*m_order).sweepIndex = 0;
        ctx.tickBT();
    }

    std::string getName() const override { return "Start Clean"; }
    EventType getType() const override { return EventType::CLEAN_START; }
};

}  // namespace des
