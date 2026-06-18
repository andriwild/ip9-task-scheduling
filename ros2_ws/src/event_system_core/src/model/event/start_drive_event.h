#pragma once

#include <utility>

#include "base.h"
#include "stop_drive_event.h"
#include "../i_sim_context.h"
#include "../../plugins/order_registry.h"
#include "../robot.h"

class StartDriveEvent final : public IEvent {
    std::string location;
public:
    explicit StartDriveEvent(const int time, std::string location)
        : IEvent(time)
        , location(std::move(location))
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartDriveEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->setCharging(false);

        if (ctx.getRobot()->getLocation() == location) {
            ctx.startActivity(std::make_shared<StopDriveEvent>(time, location, 0));
        }
        auto [duration, distance] = ctx.scheduleArrival(location);
        ctx.startActivity(std::make_shared<StopDriveEvent>(static_cast<int>(time + duration), location, distance));
        ctx.getRobot()->setDriving(true);
        ctx.getRobot()->setTargetLocation(location);

        auto orderPtr = ctx.getOrderPtr();
        if (orderPtr) {
            auto& plugin = OrderRegistry::instance().get(orderPtr->type);
            plugin.onStartDriveEvent(ctx, *orderPtr);
        }

        ctx.notifyEvent(*this);
        ctx.notifyBatteryChanged();
    }

    std::string getName() const override { return "Departing: " + location; }
    des::EventType getType() const override { return des::EventType::START_DRIVE; }
};

// Idempotent drive request: marks the robot driving and enqueues the StartDriveEvent.
// A second call while a drive is already pending is a no-op, so repeated BT ticks
// within the same simulation instant cannot enqueue duplicate drives.
inline void requestDrive(ISimContext& ctx, const std::string& target) {
    const auto robot = ctx.getRobot();
    if (robot->isDriving()) {
        return;
    }
    robot->setDriving(true);
    robot->setTargetLocation(target);
    ctx.pushEvent(std::make_shared<StartDriveEvent>(ctx.getTime(), target));
}
