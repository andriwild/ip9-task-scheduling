#pragma once

#include "model/event/base.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "../../plugins/order_registry.h"

class StopDriveEvent final : public IEvent {
public:
    double distance;
    std::string location;

    explicit StopDriveEvent(const int time, const std::string &location, const double distance)
        : IEvent(time)
        , distance(distance)
        , location(location)
    {}

    void execute(ISimContext& ctx) override {
        ctx.robotMoved(this->location, this->distance);
        ctx.getRobot()->setDriving(false);
        ctx.setBTBlackboard("location", this->location);

        auto orderPtr = ctx.getOrderPtr();
        if (orderPtr) {
            auto& plugin = OrderRegistry::instance().get(orderPtr->type);
            plugin.onStopDriveEvent(ctx, *orderPtr);
        }

        ctx.notifyEvent(*this);

        ctx.tickBT();
    }

    std::string getName() const override { return "Arrived: " + location; }
    des::EventType getType() const override { return des::EventType::STOP_DRIVE; }
};
