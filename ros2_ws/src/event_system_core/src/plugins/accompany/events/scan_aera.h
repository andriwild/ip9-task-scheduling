#pragma once

#include <cassert>

#include "model/event/base.h"
#include "scan_complete.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "util/rnd.h"
#include "plugins/accompany/accompany_order.h"

class ScanAera final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit ScanAera(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<ScanAera>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.notifyEvent(*this);
        ctx.getRobot()->setScanning(true);

        const auto& accompany = static_cast<const AccompanyOrder&>(*m_order);
        const auto& personName = accompany.personName;
        const std::string personLocation = ctx.getPersonLocation(personName);
        const std::string robotLocation  = ctx.getRobot()->getLocation();
        const bool personPresent = robotLocation == personLocation;
        const double areaToSearch = ctx.getLocationArea(robotLocation);
        const double fieldOfView  = ctx.getConfig()->personDetectionRange * ctx.getConfig()->personDetectionRange;
        assert(fieldOfView != 0);
        const double steps = (areaToSearch / fieldOfView) + 1;
        const int scanTime = steps * (2 * ctx.getConfig()->personDetectionRange / ctx.getConfig()->robotSpeed);
        assert(scanTime != 0);
        const int foundAt = rnd::uni(ctx.rng(), 1, scanTime);

        if (personPresent) {
            ctx.startActivity(std::make_shared<ScanComplete>(this->time + foundAt, m_order, personPresent, scanTime - foundAt));
        } else {
            ctx.startActivity(std::make_shared<ScanComplete>(this->time + scanTime, m_order, personPresent, 0));
        }
    }

    std::string getName() const override { return "Scan Area"; }
    des::EventType getType() const override { return des::EventType::SCAN_AREA; }
};
