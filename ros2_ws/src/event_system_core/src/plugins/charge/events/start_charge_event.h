#pragma once

#include <cassert>

#include "model/event/base.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "plugins/i_order.h"
#include "end_charge_event.h"

class StartChargeEvent final : public IEvent {
    des::OrderPtr m_order;
public:
    explicit StartChargeEvent(const int time, const des::OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartChargeEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        m_order->state = des::MissionState::IN_PROGRESS;
        ctx.notifyEvent(*this);

        assert(ctx.getRobot()->getLocation() == ctx.getRobot()->getIdleLocation());
        const double netChargingPower = ctx.getConfig()->chargingRate - ctx.getConfig()->energyConsumptionBase;
        const double timeToFull       = ctx.getRobot()->m_bat->timeToFull(netChargingPower);

        ctx.startActivity(std::make_shared<EndChargeEvent>(static_cast<int>(this->time + timeToFull), m_order));
    }

    std::string getName() const override { return "Start Charge"; }
    des::EventType getType() const override { return des::EventType::CHARGE_MISSION_START; }
};
