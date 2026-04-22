#pragma once

#include <cassert>

#include "model/event/base.h"
#include "scan_complete.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "util/rnd.h"
#include "plugins/accompany/accompany_order.h"

class ScanAera final : public IEvent {
public:
    explicit ScanAera(const int time) : IEvent(time) {}

    void execute(ISimContext& ctx) override {
        ctx.notifyEvent(*this);
        ctx.getRobot()->setScanning(true);

        const auto& accompany = static_cast<const AccompanyOrder&>(*ctx.getOrderPtr());
        const auto& personName = accompany.personName;
        const std::string personLocation = ctx.getPersonLocation(personName);
        const std::string robotLocation  = ctx.getRobot()->getLocation();
        const bool personPresent = robotLocation == personLocation;
        const double areaToSearch = ctx.getSearchArea(robotLocation);
        const double fieldOfView  = ctx.getConfig()->personDetectionRange * ctx.getConfig()->personDetectionRange;
        assert(fieldOfView != 0);
        const double steps = (areaToSearch / fieldOfView) + 1;
        const int scanTime = steps * (2 * ctx.getConfig()->personDetectionRange / ctx.getConfig()->robotSpeed);
        assert(scanTime != 0);
        const int foundAt = rnd::uni(ctx.rng(), 1, scanTime);

        if (personPresent) {
            ctx.pushEvent(std::make_shared<ScanComplete>(this->time + foundAt, personPresent, scanTime - foundAt));
        } else {
            ctx.pushEvent(std::make_shared<ScanComplete>(this->time + scanTime, personPresent, 0));
        }
    }

    std::string getName() const override { return "Scan Area"; }
    des::EventType getType() const override { return des::EventType::SCAN_AREA; }
};
