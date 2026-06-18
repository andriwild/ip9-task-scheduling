#pragma once

#include "model/event/base.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "plugins/accompany/accompany_order.h"

class ScanComplete final : public IEvent {
    des::OrderPtr m_order;
public:
    int m_remainigSearchTime;
    bool m_found;
    explicit ScanComplete(const int time, const des::OrderPtr& order, const bool found, const int remainingSearchTime)
        : IEvent(time)
        , m_order(order)
        , m_remainigSearchTime(remainingSearchTime)
        , m_found(found)
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<ScanComplete>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        const auto& accompany = static_cast<const AccompanyOrder&>(*m_order);
        const auto& personName = accompany.personName;
        const std::string personLocation = ctx.getPersonLocation(personName);
        const std::string robotLocation  = ctx.getRobot()->getLocation();
        const bool personPresent = robotLocation == personLocation;
        if (personPresent) {
            ctx.getRobot()->setIsPersonVisible(true);
            auto person = ctx.getPersonByName(personName);
            person->busy = true;
        } else if (this->m_remainigSearchTime > 0) {
            ctx.startActivity(std::make_shared<ScanComplete>(this->time + this->m_remainigSearchTime, m_order, personPresent, 0));
            return;
        }
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Scan Complete"; }
    des::EventType getType() const override { return des::EventType::SCAN_COMPLETE; }
};
