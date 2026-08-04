#pragma once

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/person.h"
#include "model/robot.h"
#include "plugins/accompany/accompany_order.h"
#include "util/log.h"

class ScanPointEvent final : public IEvent {
    des::OrderPtr m_order;

public:
    explicit ScanPointEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<ScanPointEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        const auto& accompany = static_cast<const AccompanyOrder&>(*m_order);
        const auto& personName = accompany.personName;
        const bool seen = ctx.robotSeesPerson(personName);

        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany.search"), "ScanPoint t=%d room=%s point=%zu person=%s seen=%d", this->time, ctx.getRobot()->getLocation().c_str(), accompany.scanIndex, personName.c_str(), seen);

        if (seen) {
            ctx.getRobot()->setIsPersonVisible(true);
            if (auto* person = ctx.getPersonByName(personName)) {
                person->busy = true;
            }
        }
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Scan"; }
    des::EventType getType() const override { return des::EventType::SCAN; }
};
