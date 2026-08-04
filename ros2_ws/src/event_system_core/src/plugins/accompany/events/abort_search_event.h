#pragma once

#include <algorithm>

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/order.h"
#include "plugins/accompany/accompany_order.h"
#include "plugins/accompany/search_exclusion.h"
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
                DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.search"), "Abort Search for %s: person is OUTSIDE the building", personName.c_str());
            } else {
                const bool unreachable = isSearchExcluded(ctx.getConfig()->searchExcludedRooms, loc);
                accompany->abortReason = unreachable ? SearchAbortReason::IN_BUILDING_UNREACHABLE
                                                     : SearchAbortReason::IN_BUILDING_FINDABLE;
                const auto& plan    = accompany->plannedSearch;
                const bool searched = std::find(plan.begin(), plan.end(), loc) != plan.end();
                DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.search"),
                             "Abort Search for %s: person was IN BUILDING at %s (%s)",
                             personName.c_str(), loc.c_str(),
                             unreachable ? "unreachable room (excluded from search)" : (searched ? "searched room, timing miss" : "unsearched room, belief miss"));
            }
        }
        ctx.notifyEvent(*this);
    }

    std::string getName() const override {
        if (auto* a = dynamic_cast<AccompanyOrder*>(m_order.get())) {
            switch (a->abortReason) {
                case SearchAbortReason::OUTSIDE:                 return "Abort Search: person outside";
                case SearchAbortReason::IN_BUILDING_UNREACHABLE: return "Abort Search: unreachable room";
                case SearchAbortReason::IN_BUILDING_FINDABLE:    return "Abort Search: missed in building";
                default: break;
            }
        }
        return "Abort Search";
    }

    //TODO: magic color strings
    std::string getColor() const override {
        if (auto* a = dynamic_cast<AccompanyOrder*>(m_order.get())) {
            switch (a->abortReason) {
                case SearchAbortReason::OUTSIDE:                 return "#9aa4b0";
                case SearchAbortReason::IN_BUILDING_UNREACHABLE: return "#d0a020";
                case SearchAbortReason::IN_BUILDING_FINDABLE:    return "#d62728";
                default: break;
            }
        }
        return "";
    }

    des::EventType getType() const override { return des::EventType::ABORT_SEARCH; }
};
