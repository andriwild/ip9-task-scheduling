#pragma once

#include <algorithm>
#include <cmath>

#include "model/event/base.h"
#include "scan_complete.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "util/log.h"
#include "plugins/accompany/accompany_order.h"

class Scan final : public IEvent {
    des::OrderPtr m_order;
    size_t m_index;
    int m_startTime;

public:
    explicit Scan(const int time, const des::OrderPtr& order, const size_t index, const int startTime)
        : IEvent(time)
        , m_order(order)
        , m_index(index)
        , m_startTime(startTime)
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<Scan>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        const auto& accompany = static_cast<const AccompanyOrder&>(*m_order);
        const std::string& personName = accompany.personName;
        const std::string room = ctx.getRobot()->getLocation();
        const des::RoomTour* tour = ctx.roomTour(room);

        if (tour == nullptr || m_index >= tour->m_path.size()) {
            ctx.startActivity(std::make_shared<ScanComplete>(this->time, m_order, ctx.robotSeesPerson(personName), 0));
            return;
        }

        const des::Point& p = tour->m_path[m_index];
        ctx.robotMovedTo(p);
        ctx.notifyEvent(*this);

        const bool present = ctx.robotSeesPerson(personName);
        const auto pos = ctx.getPersonPosition(personName);
        const double radius = ctx.getConfig()->personDetectionRange;
        double dist = -1.0;
        if (pos) {
            dist = std::hypot(pos->m_x - p.m_x, pos->m_y - p.m_y);
        }
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.search"),
            "Scan t=%d room=%s point=%zu/%zu at (%.2f,%.2f) | personInRoom=%d personPos=(%.2f,%.2f) dist=%.2f radius=%.2f",
            this->time, room.c_str(), m_index, tour->m_path.size(), p.m_x, p.m_y,
            present, pos ? pos->m_x : 0.0, pos ? pos->m_y : 0.0, dist, radius);

        if (present && pos && dist <= radius) {
            DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.search"), "Scan: FOUND %s at point %zu (dist %.2f <= %.2f)", personName.c_str(), m_index, dist, radius);
            ctx.startActivity(std::make_shared<ScanComplete>(this->time, m_order, true, 0));
            return;
        }

        if (m_index + 1 >= tour->m_path.size()) {
            DES_LOG_INFO(rclcpp::get_logger("des.plugin.accompany.search"), "Scan: last point reached, %s not found geometrically (personInRoom=%d)", personName.c_str(), present);
            ctx.startActivity(std::make_shared<ScanComplete>(this->time, m_order, present, 0));
            return;
        }

        double cumulative = 0.0;
        for (size_t k = 1; k <= m_index + 1; ++k) {
            const des::Point& a = tour->m_path[k - 1];
            const des::Point& b = tour->m_path[k];
            cumulative += std::hypot(b.m_x - a.m_x, b.m_y - a.m_y);
        }
        const double speed = ctx.getConfig()->robotSpeed;
        const int nextTime = std::max(this->time, m_startTime + static_cast<int>(std::lround(cumulative / speed)));
        ctx.startActivity(std::make_shared<Scan>(nextTime, m_order, m_index + 1, m_startTime));
    }

    std::string getName() const override { return "Scan"; }
    des::EventType getType() const override { return des::EventType::SCAN; }
};
