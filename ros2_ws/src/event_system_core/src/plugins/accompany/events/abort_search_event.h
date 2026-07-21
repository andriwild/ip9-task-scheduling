#pragma once

#include <algorithm>

#include "model/event/base.h"
#include "model/event/mission_complete_event.h"
#include "model/i_sim_context.h"
#include "model/robot_state.h"
#include "plugins/i_order.h"
#include "plugins/accompany/accompany_order.h"
#include "util/log.h"

class AbortSearchEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit AbortSearchEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<AbortSearchEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        if (auto* accompany = dynamic_cast<AccompanyOrder*>(m_order.get())) {
            const auto& personName = accompany->personName;
            const std::string loc  = ctx.getPersonLocation(personName);
            if (loc == "OUTDOOR") {
                accompany->abortReason = SearchAbortReason::OUTSIDE;
                DES_LOG_INFO(rclcpp::get_logger("des.plugin.accompany.search"),
                             "Abort Search for %s: person is OUTSIDE the building", personName.c_str());
            } else {
                accompany->abortReason = SearchAbortReason::IN_BUILDING;
                const auto& plan    = accompany->plannedSearch;
                const bool searched = std::find(plan.begin(), plan.end(), loc) != plan.end();
                DES_LOG_INFO(rclcpp::get_logger("des.plugin.accompany.search"),
                             "Abort Search for %s: person was IN BUILDING at %s (%s)",
                             personName.c_str(), loc.c_str(),
                             searched ? "searched room, timing miss" : "unsearched room, belief miss");
            }
        }
        m_order->state = des::MissionState::FAILED;
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, m_order));
        ctx.notifyEvent(*this);
    }

    std::string getName() const override {
        if (auto* a = dynamic_cast<AccompanyOrder*>(m_order.get())) {
            if (a->abortReason == SearchAbortReason::OUTSIDE) {
                return "Abort Search: person outside";
            }
            if (a->abortReason == SearchAbortReason::IN_BUILDING) {
                return "Abort Search: missed in building";
            }
        }
        return "Abort Search";
    }

    std::string getColor() const override {
        if (auto* a = dynamic_cast<AccompanyOrder*>(m_order.get())) {
            if (a->abortReason == SearchAbortReason::OUTSIDE) {
                return "#9aa4b0";
            }
            if (a->abortReason == SearchAbortReason::IN_BUILDING) {
                return "#d62728";
            }
        }
        return "";
    }

    des::EventType getType() const override { return des::EventType::ABORT_SEARCH; }
};
