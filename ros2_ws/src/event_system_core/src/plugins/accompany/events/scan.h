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
public:
    enum class Phase { Drive, Arrive, Detect };

private:
    des::OrderPtr m_order;
    size_t m_index;
    Phase m_phase;

public:
    explicit Scan(const int time, const des::OrderPtr& order, const size_t index, const Phase phase)
        : IEvent(time)
        , m_order(order)
        , m_index(index)
        , m_phase(phase)
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

        switch (m_phase) {
            case Phase::Drive: {
                const des::Point from = ctx.getRobot()->getPosition();
                const double dist = std::hypot(p.m_x - from.m_x, p.m_y - from.m_y);
                const double speed = ctx.getConfig()->robotSpeed;
                const int travel = std::max(1, static_cast<int>(std::lround(dist / speed)));
                ctx.getRobot()->setDriving(true);
                ctx.notifyEvent(*this);
                ctx.startActivity(std::make_shared<Scan>(this->time + travel, m_order, m_index, Phase::Arrive));
                break;
            }
            case Phase::Arrive: {
                ctx.robotMovedTo(p);
                ctx.getRobot()->setDriving(false);
                ctx.notifyEvent(*this);
                ctx.startActivity(std::make_shared<Scan>(this->time, m_order, m_index, Phase::Detect));
                break;
            }
            case Phase::Detect: {
                ctx.notifyEvent(*this);
                const bool present = ctx.robotSeesPerson(personName);
                const auto pos = ctx.getPersonPosition(personName);
                const double radius = ctx.getConfig()->personDetectionRange;
                bool found = false;
                if (present && pos) {
                    const double dist = std::hypot(pos->m_x - p.m_x, pos->m_y - p.m_y);
                    found = dist <= radius;
                }
                DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.search"),
                    "Scan t=%d room=%s point=%zu/%zu found=%d", this->time, room.c_str(), m_index, tour->m_path.size(), found);

                if (found) {
                    ctx.startActivity(std::make_shared<ScanComplete>(this->time, m_order, true, 0));
                } else if (m_index + 1 >= tour->m_path.size()) {
                    ctx.startActivity(std::make_shared<ScanComplete>(this->time, m_order, present, 0));
                } else {
                    ctx.startActivity(std::make_shared<Scan>(this->time, m_order, m_index + 1, Phase::Drive));
                }
                break;
            }
        }
    }

    std::string getName() const override {
        switch (m_phase) {
            case Phase::Drive:  return "Scan Drive";
            case Phase::Arrive: return "Scan Arrive";
            case Phase::Detect: return "Scan";
        }
        return "Scan";
    }

    des::EventType getType() const override {
        switch (m_phase) {
            case Phase::Drive:  return des::EventType::START_DRIVE;
            case Phase::Arrive: return des::EventType::STOP_DRIVE;
            case Phase::Detect: return des::EventType::SCAN;
        }
        return des::EventType::SCAN;
    }
};
