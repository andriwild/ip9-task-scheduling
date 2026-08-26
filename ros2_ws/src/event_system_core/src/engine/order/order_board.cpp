#include "engine/order/order_board.h"

#include <algorithm>
#include <format>
#include <string>
#include <utility>

#include "engine/contracts/estimation_view.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"
#include "engine/event_queue.h"
#include "engine/event/mission_dispatch_event.h"
#include "observer/event_bus.h"
#include "plugins/order_registry.h"
#include "plugins/charge/charge_order.h"
#include "util/log.h"

namespace des {

OrderBoard::OrderBoard(EventQueue& queue, EventBus& bus)
    : m_queue(queue)
    , m_bus(bus)
{
}

void OrderBoard::setCurrent(const OrderPtr& orderPtr) {
    if (orderPtr) {
        DES_LOG_DEBUG("des.context.mission", "Current mission set: %d (type=%s)", orderPtr->id, orderPtr->type.c_str());
    } else if (m_current) {
        DES_LOG_DEBUG("des.context.mission", "Current mission cleared (was %d)", m_current->id);
    }
    m_current = orderPtr;
}

OrderPtr OrderBoard::current() const {
    return m_current;
}

OrderPtr OrderBoard::effective() const {
    if (auto interrupt = m_interrupt.active()) {
        return interrupt;
    }
    return m_current;
}

void OrderBoard::updateState(const OrderState& newState) {
    assert(m_current != nullptr);
    DES_LOG_DEBUG("des.context.mission", "Mission %d (type=%s) state: %s -> %s", m_current->id, m_current->type.c_str(), orderStateStr(m_current->state).c_str(), orderStateStr(newState).c_str());
    m_current->state = newState;
}

void OrderBoard::complete(ISimContext& ctx, const OrderPtr& order) {
    assert(order != nullptr);
    if (order->type != kChargeOrderType) {
        const int deadline = order->deadline.value_or(ctx.getTime());
        const int timeDiff = ctx.getTime() - deadline;

        const std::string detail = OrderRegistry::instance().get(order->type).outcomeDetail(*order);
        std::string summary = orderStateStr(order->state);
        if (!detail.empty()) {
            summary += " (" + detail + ")";
        }
        if (order->deadline.has_value()) {
            summary += std::format(" [{:+d}s]", timeDiff);
        }
        const bool routine = order->execution != ExecutionMode::SCHEDULED
            && order->state == OrderState::COMPLETED;
        if (routine) {
            DES_LOG_DEBUG("des.mission", "Mission %d (%s) %s", order->id, order->type.c_str(), summary.c_str());
        } else {
            DES_LOG_INFO("des.mission", "Mission %d (%s) %s", order->id, order->type.c_str(), summary.c_str());
        }
    }
    if (m_current == order) {
        setCurrent(nullptr);
    }
}

void OrderBoard::setDispatchPlan(const OrderList& orders) {
    m_dispatchPlan.assign(orders.begin(), orders.end());
    std::ranges::stable_sort(m_dispatchPlan, {}, [](const OrderPtr& order) {
        return order->dispatchTime;
    });
}

void OrderBoard::addScheduled(const OrderPtr& orderPtr) {
    std::erase(m_dispatchPlan, orderPtr);
    m_scheduled.add(orderPtr);
}

bool OrderBoard::hasScheduled() const {
    return m_scheduled.has();
}

OrderPtr OrderBoard::nextScheduled() {
    return m_scheduled.peek();
}

OrderPtr OrderBoard::popScheduled() {
    return m_scheduled.pop();
}

std::optional<int> OrderBoard::nextScheduledDispatchTime() const {
    if (m_dispatchPlan.empty()) {
        return std::nullopt;
    }
    return m_dispatchPlan.front()->dispatchTime;
}

OrderPtr OrderBoard::peekNextScheduledOrder() const {
    if (m_dispatchPlan.empty()) {
        return nullptr;
    }
    return m_dispatchPlan.front();
}

std::vector<OrderPtr> OrderBoard::peekScheduledOrdersUntil(const int untilTime) const {
    std::vector<OrderPtr> orders;
    for (const auto& order : m_dispatchPlan) {
        if (order->dispatchTime > untilTime) {
            break;
        }
        orders.push_back(order);
    }
    return orders;
}

void OrderBoard::addBackground(const OrderPtr& orderPtr) {
    m_background.add(orderPtr);
}

bool OrderBoard::hasBackground() const {
    return m_background.has();
}

OrderPtr OrderBoard::acceptFeasibleBackground(ISimContext& ctx) {
    if (!m_background.hasPlanned()) {
        m_background.plan(ctx);
    }
    auto accepted = m_background.popPlanned();
    if (accepted) {
        m_current = accepted;
    }
    return accepted;
}

bool OrderBoard::pushInterrupt(ISimContext& ctx, const OrderPtr& order) {
    assert(order->execution == ExecutionMode::INTERRUPT && "Interrupt pushed with wrong ExecutionMode");
    const auto robot = ctx.getRobot();
    if (robot->isBatteryLow()) {
        DES_LOG_DEBUG("des.context.interrupt", "Reject %d (type=%s) — battery low (SoC %.0f%%), heading to dock", order->id, order->type.c_str(), robot->batteryStats().soc * 100.0);
        return false;
    }
    if (!m_interrupt.push(order, m_current)) {
        return false;
    }

    // Shifts the in-flight activity-end event by the interrupt's duration
    const bool wasDriving = robot->isDriving();
    const bool continuesDuringInterrupt = !wasDriving
        && robot->getLocation() == robot->getIdleLocation()
        && robot->getState()->chargesAtDock();

    auto& plugin = OrderRegistry::instance().get(order->type);
    const int duration = static_cast<int>(plugin.estimateMissionDuration(*order, ctx, robot->getLocation()));

    if (auto e = robot->inFlight().lock()) {
        if (continuesDuringInterrupt) {
            DES_LOG_DEBUG("des.context.interrupt", "Push %d (type=%s, dur=%ds) at t=%d — kept in-flight '%s' at %d, charging continues at the dock", order->id, order->type.c_str(), duration, ctx.getTime(), e->getName().c_str(), e->time);
        } else {
            const int oldTime = e->time;
            const int newTime = oldTime + duration;
            e->cancelled = true;
            auto shifted = e->withTime(newTime);
            ctx.startActivity(shifted);
            DES_LOG_DEBUG("des.context.interrupt", "Push %d (type=%s, dur=%ds) at t=%d — shifted in-flight '%s': %d → %d", order->id, order->type.c_str(), duration, ctx.getTime(), e->getName().c_str(), oldTime, newTime);
        }
    } else {
        DES_LOG_DEBUG("des.context.interrupt", "Push %d (type=%s, dur=%ds) at t=%d — robot idle, no in-flight to shift", order->id, order->type.c_str(), duration, ctx.getTime());
    }

    if (wasDriving) {
        robot->setDriving(false);
        m_bus.notifyEvent(ctx.getTime(), EventType::STOP_DRIVE, "Drive paused: " + robot->getLocation(), false, robot->isCharging(), "", order->id);
    }

    m_interrupt.suspend(wasDriving);
    plugin.onMissionStart(ctx, *order);
    return true;
}

void OrderBoard::popInterrupt(ISimContext& ctx, const OrderPtr& completedOrder) {
    const auto robot = ctx.getRobot();
    m_interrupt.pop(completedOrder);
    bool resumeDrive = false;
    if (auto snap = m_interrupt.takeSuspended()) {
        resumeDrive = snap->wasDriving;
    }
    if (m_current) {
        OrderRegistry::instance().get(m_current->type).onMissionResume(ctx, *m_current);
    } else {
        ctx.changeRobotState(std::make_unique<IdleState>());
    }
    if (resumeDrive && robot->getLocation() != robot->getTargetLocation()) {
        robot->setDriving(true);
        m_bus.notifyEvent(ctx.getTime(), EventType::START_DRIVE, "Drive resumed: " + robot->getTargetLocation(), true, robot->isCharging(), "", -1);
    }
    if (ctx.getConfig()->replanBackgroundOnInterrupt) {
        m_background.invalidatePlan();
    }
    DES_LOG_DEBUG("des.context.interrupt", "Pop %d (type=%s) at t=%d — resuming main mission", completedOrder->id, completedOrder->type.c_str(), ctx.getTime());
}

bool OrderBoard::hasActiveInterrupt() const {
    return m_interrupt.has();
}

void OrderBoard::reset() {
    DES_LOG_DEBUG("des.context.mission", "Reset (pending=%zu, background=%zu, interrupt=%s cleared)", m_scheduled.size(), m_background.size(), m_interrupt.has() ? "yes" : "no");
    m_current = nullptr;
    m_scheduled.clear();
    m_background.clear();
    m_interrupt.clear();
    m_dispatchPlan.clear();
}

}  // namespace des
