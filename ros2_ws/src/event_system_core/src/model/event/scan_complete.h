#pragma once

#include "base.h"
#include "../i_sim_context.h"
#include "../robot.h"
#include "../../plugins/accompany/accompany_order.h"

class ScanComplete final : public IEvent {
public:
    int m_remainigSearchTime;
    bool m_found;
    explicit ScanComplete(const int time, const bool found, const int remainingSearchTime)
        : IEvent(time)
        , m_remainigSearchTime(remainingSearchTime)
        , m_found(found)
    {}

    void execute(ISimContext& ctx) override {
        const auto& accompany = static_cast<const AccompanyOrder&>(*ctx.getOrderPtr());
        const auto& personName = accompany.personName;
        const std::string personLocation = ctx.getPersonLocation(personName);
        const std::string robotLocation  = ctx.getRobot()->getLocation();
        const bool personPresent = robotLocation == personLocation;
        if (personPresent) {
            ctx.getRobot()->setIsPersonVisible(true);
        } else if (this->m_remainigSearchTime > 0) {
            ctx.pushEvent(std::make_shared<ScanComplete>(this->time + this->m_remainigSearchTime, personPresent, 0));
            return;
        }
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Scan Complete"; }
    des::EventType getType() const override { return des::EventType::SCAN_COMPLETE; }
};
