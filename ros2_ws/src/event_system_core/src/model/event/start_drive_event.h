#pragma once

#include <memory>
#include <utility>

#include "base.h"
#include "stop_drive_event.h"
#include "drive_target.h"
#include "../i_sim_context.h"
#include "../robot.h"
#include "../../util/types.h"

class StartDriveEvent final : public IEvent {
    std::shared_ptr<DriveTarget> m_target;
    std::shared_ptr<IEvent> m_onArrive;

public:
    explicit StartDriveEvent(const int time, std::shared_ptr<DriveTarget> target, std::shared_ptr<IEvent> onArrive = nullptr)
        : IEvent(time)
        , m_target(std::move(target))
        , m_onArrive(std::move(onArrive))
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartDriveEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->setCharging(false);

        const auto [duration, distance] = m_target->travel(ctx);
        ctx.getRobot()->setDriving(true);
        m_target->onStart(ctx);
        ctx.startActivity(std::make_shared<StopDriveEvent>(this->time + duration, m_target, distance, m_onArrive));

        ctx.notifyEvent(*this);
        ctx.notifyBatteryChanged();
    }

    std::string getName() const override { return "Departing: " + m_target->label(); }
    des::EventType getType() const override { return des::EventType::START_DRIVE; }
};

inline void requestDrive(ISimContext& ctx, const std::string& target) {
    const auto robot = ctx.getRobot();
    if (robot->isDriving()) {
        return;
    }
    robot->setDriving(true);
    robot->setTargetLocation(target);
    ctx.pushEvent(std::make_shared<StartDriveEvent>(ctx.getTime(), std::make_shared<RoomTarget>(target)));
}

inline void requestDrive(ISimContext& ctx, const des::Point& target, const des::Polygon& visibility, std::shared_ptr<IEvent> onArrive = nullptr) {
    ctx.startActivity(std::make_shared<StartDriveEvent>(ctx.getTime(), std::make_shared<PointTarget>(target, visibility), std::move(onArrive)));
}
