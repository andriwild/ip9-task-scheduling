#pragma once

#include <cassert>

#include "engine/contracts/i_event.h"
#include "engine/event/charge_phase_transition_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/order.h"
#include "end_charge_event.h"

namespace des {

class StartChargeEvent final : public IEvent {
    OrderPtr m_order;
public:
    explicit StartChargeEvent(const int time, const OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartChargeEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        m_order->state = MissionState::IN_PROGRESS;
        ctx.getRobot()->beginChargeSession(this->time);
        ctx.notifyEvent(*this);

        assert(ctx.getRobot()->getLocation() == ctx.getRobot()->getIdleLocation());
        const double netChargingPower = ctx.getConfig()->chargingRate - ctx.getConfig()->energyConsumptionBase;
        const double timeToFull       = ctx.getRobot()->batteryTimeToFull(netChargingPower);

        const double timeToTransition = ctx.getRobot()->batteryTimeToPhaseTransition(netChargingPower);
        if (timeToTransition >= 0.0) {
            ctx.pushEvent(std::make_shared<ChargePhaseTransitionEvent>(static_cast<int>(this->time + timeToTransition)));
        }

        ctx.startActivity(std::make_shared<EndChargeEvent>(static_cast<int>(this->time + timeToFull), m_order));
    }

    std::string getName() const override { return "Start Charge"; }
    EventType getType() const override { return EventType::CHARGE_MISSION_START; }
};

}  // namespace des
