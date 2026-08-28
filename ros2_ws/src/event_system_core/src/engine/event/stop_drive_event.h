#pragma once

#include <memory>
#include <utility>

#include "engine/contracts/i_event.h"
#include "engine/event/drive_target.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "util/point.h"

namespace des {

class StopDriveEvent final : public IEvent {
    std::shared_ptr<DriveTarget> m_target;
    double m_distance;
    std::shared_ptr<IEvent> m_onArrive;

public:
    // Filled on arrival, read by the move trace.
    std::string m_room;
    std::optional<Point> m_at;

    explicit StopDriveEvent(const int time, std::shared_ptr<DriveTarget> target, const double distance, std::shared_ptr<IEvent> onArrive = nullptr)
        : IEvent(time)
        , m_target(std::move(target))
        , m_distance(distance)
        , m_onArrive(std::move(onArrive))
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StopDriveEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        m_target->arrive(ctx, m_distance);
        m_room = ctx.getRobot()->getLocation();
        m_at = ctx.getRobot()->getPosition();
        ctx.getRobot()->setDriving(false);
        ctx.notifyEvent(*this);
        ctx.notifyRobotStateChanged();

        if (m_onArrive) {
            ctx.startActivity(m_onArrive->withTime(this->time));
        } else {
            ctx.tickBT();
        }
    }

    std::string getName() const override { return "Arrived: " + m_target->label(); }
    EventType getType() const override { return EventType::STOP_DRIVE; }
    double getDistance() const override { return m_distance; }
};

}  // namespace des
