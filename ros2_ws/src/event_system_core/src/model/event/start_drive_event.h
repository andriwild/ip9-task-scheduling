#pragma once

#include <cassert>
#include <utility>

#include "base.h"
#include "stop_drive_event.h"
#include "../i_sim_context.h"
#include "../robot.h"

class StartDriveEvent final : public IEvent {
    std::string location;
public:
    explicit StartDriveEvent(const int time, std::string location)
        : IEvent(time)
        , location(std::move(location))
    {}

    void execute(ISimContext& ctx) override {
        assert(!ctx.getRobot()->isDriving());
        ctx.getRobot()->setCharging(false);

        if (ctx.getRobot()->getLocation() == location) {
            ctx.pushEvent(std::make_shared<StopDriveEvent>(time, location, 0));
        }
        auto [duration, distance] = ctx.scheduleArrival(location);
        ctx.pushEvent(std::make_shared<StopDriveEvent>(static_cast<int>(time + duration), location, distance));
        ctx.getRobot()->setDriving(true);
        ctx.getRobot()->setTargetLocation(location);
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Departing: " + location; }
    des::EventType getType() const override { return des::EventType::START_DRIVE; }
};
