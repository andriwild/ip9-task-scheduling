#pragma once

#include <format>
#include <rclcpp/rclcpp.hpp>

#include "../../util/log.h"
#include "base.h"
#include "../../plugins/order_registry.h"
#include "../i_sim_context.h"
#include "util/types.h"

class MissionCompleteEvent final : public IEvent {
public:
    des::OrderPtr orderPtr;
    explicit MissionCompleteEvent(const int time, const des::OrderPtr& order)
        : IEvent(time)
        , orderPtr(order)
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<MissionCompleteEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& plugin = OrderRegistry::instance().get(orderPtr->type);

        std::string execStr;
        switch (orderPtr->execution) {
            case des::ExecutionMode::SCHEDULED:  execStr = "scheduled"; break;
            case des::ExecutionMode::BACKGROUND: execStr = "background"; break;
            case des::ExecutionMode::INTERRUPT:  execStr = "interrupt"; break;
        };

        DES_LOG_INFO(rclcpp::get_logger("des.event.mission_complete"), "id=%d type=%s state=%s exec=%s — onMissionEnd", orderPtr->id, orderPtr->type.c_str(), des::missionStateStr(orderPtr->state).c_str(), execStr.c_str());
        plugin.onMissionEnd(ctx, *orderPtr);
        if (orderPtr->execution != des::ExecutionMode::INTERRUPT) {
            ctx.completeOrder(this->orderPtr);
        }
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override {
        return std::format("Mission {} Complete: {}", orderPtr->id, des::missionStateStr(orderPtr->state));
    }
    des::EventType getType() const override { return des::EventType::MISSION_COMPLETE; }
    int getMissionId() const override { return orderPtr ? orderPtr->id : -1; }
};
