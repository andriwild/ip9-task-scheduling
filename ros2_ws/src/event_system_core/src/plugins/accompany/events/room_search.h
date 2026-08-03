#pragma once

#include "model/event/base.h"
#include "model/event/start_drive_event.h"
#include "scan.h"
#include "scan_complete.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "util/log.h"
#include "plugins/accompany/accompany_order.h"
#include "plugins/accompany/search_exclusion.h"

class RoomSearch final : public IEvent {
    des::OrderPtr m_order;

public:
    explicit RoomSearch(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<RoomSearch>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.notifyEvent(*this);
        ctx.getRobot()->setScanning(true);

        const auto& accompany = static_cast<const AccompanyOrder&>(*m_order);
        const std::string& personName = accompany.personName;
        const std::string room = ctx.getRobot()->getLocation();

        if (isSearchExcluded(ctx.getConfig()->searchExcludedRooms, room)) {
            DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.search"), "RoomSearch t=%d room='%s' is excluded from search; skipping scan", this->time, room.c_str());
            ctx.startActivity(std::make_shared<ScanComplete>(this->time, m_order, ctx.robotSeesPerson(personName), 0));
            return;
        }

        const des::RoomTour& tour = ctx.room(room).m_tour;
        if (tour.empty()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.plugin.accompany.search"), "RoomSearch t=%d No tour for room '%s'; skipping scan", this->time, room.c_str());
            ctx.startActivity(std::make_shared<ScanComplete>(this->time, m_order, ctx.robotSeesPerson(personName), 0));
            return;
        }

        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.search"), "RoomSearch t=%d room=%s person=%s -> tour with %zu points (%.2fm)", this->time, room.c_str(), personName.c_str(), tour.m_path.size(), tour.m_distance);
        ctx.robotMovedTo(ctx.room(room).m_p);
        requestDrive(ctx, tour.m_path[0], tour.visibilityAt(0), std::make_shared<Scan>(this->time, m_order, 0));
    }

    std::string getName() const override { return "Room Search"; }
    des::EventType getType() const override { return des::EventType::ROOM_SEARCH; }
};
