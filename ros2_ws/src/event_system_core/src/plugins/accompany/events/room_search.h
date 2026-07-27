#pragma once

#include <algorithm>
#include <cassert>

#include "model/event/base.h"
#include "model/event/start_drive_event.h"
#include "scan.h"
#include "scan_complete.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "util/rnd.h"
#include "util/log.h"
#include "plugins/accompany/accompany_order.h"

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
        const des::RoomTour* tour = ctx.roomTour(room);

        if (tour != nullptr && !tour->m_path.empty()) {
            DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.search"), "RoomSearch t=%d room=%s person=%s -> tour with %zu points (%.2fm)", this->time, room.c_str(), personName.c_str(), tour->m_path.size(), tour->m_distance);
            ctx.robotMovedTo(ctx.location(room).m_p);
            requestDrive(ctx, tour->m_path[0], std::make_shared<Scan>(this->time, m_order, 0));
            return;
        }

        DES_LOG_WARN(rclcpp::get_logger("des.plugin.accompany.search"), "RoomSearch t=%d No tour for room '%s'; using area-based scan estimate", this->time, room.c_str());
        const bool personPresent = ctx.robotSeesPerson(personName);
        const double areaToSearch = ctx.location(room).m_area.value_or(1.0);
        const double fieldOfView  = ctx.getConfig()->personDetectionRange * ctx.getConfig()->personDetectionRange;
        assert(fieldOfView != 0);
        const double steps = (areaToSearch / fieldOfView) + 1;
        const int scanTime = std::max(1, static_cast<int>(steps * (2 * ctx.getConfig()->personDetectionRange / ctx.getConfig()->robotSpeed)));
        const int foundAt = rnd::uni(ctx.rng(), 1, scanTime);

        if (personPresent) {
            ctx.startActivity(std::make_shared<ScanComplete>(this->time + foundAt, m_order, personPresent, scanTime - foundAt));
        } else {
            ctx.startActivity(std::make_shared<ScanComplete>(this->time + scanTime, m_order, personPresent, 0));
        }
    }

    std::string getName() const override { return "Room Search"; }
    des::EventType getType() const override { return des::EventType::ROOM_SEARCH; }
};
