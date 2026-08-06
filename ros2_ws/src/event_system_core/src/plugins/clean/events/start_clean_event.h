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

class StartCleanEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit StartCleanEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartCleanEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        m_order->state = des::MissionState::IN_PROGRESS;
        ctx.notifyEvent(*this);

        const std::string& roomName = static_cast<const CleanOrder&>(*m_order).roomName;
        const auto area             = ctx.room(roomName).m_area;
        if (!area.has_value()) {
            DES_LOG_WARN(rclcpp::get_logger("des.plugin.clean"), "Room '%s' has no area, cleaning mission %d falls back to 1m2", roomName.c_str(), m_order->id);
        }
        const double roomArea       = area.value_or(1.0);
        const double cleaningArea = cleanConfig().cleaningArea;
        assert(cleaningArea > 0.0);
        const double cleaningSide = std::sqrt(cleaningArea);
        const double steps        = (roomArea / cleaningArea) + 1;
        const int cleanTime       = static_cast<int>(steps * (2.0 * cleaningSide / ctx.getConfig()->robotSpeed));
        assert(cleanTime > 0);

        ctx.startActivity(std::make_shared<EndCleanEvent>(this->time + cleanTime, m_order));
    }

    std::string getName() const override { return "Start Clean"; }
    des::EventType getType() const override { return des::EventType::CLEAN_START; }
};
