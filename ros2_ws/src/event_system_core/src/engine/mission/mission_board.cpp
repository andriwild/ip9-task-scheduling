#include "engine/mission/mission_board.h"

#include <format>
#include <string>
#include <utility>

#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"
#include "engine/event_queue.h"
#include "engine/event/mission_dispatch_event.h"
#include "observer/event_bus.h"
#include "plugins/order_registry.h"
#include "plugins/charge/charge_order.h"
#include "util/log.h"

MissionBoard::MissionBoard(EventQueue& queue, EventBus& bus)
    : m_queue(queue)
    , m_bus(bus)
{
}

void MissionBoard::setCurrent(const des::OrderPtr& orderPtr) {
    if (orderPtr) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.context.mission"), "Current mission set: %d (type=%s)", orderPtr->id, orderPtr->type.c_str());
    } else if (m_current) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.context.mission"), "Current mission cleared (was %d)", m_current->id);
    }
    m_current = orderPtr;
}

des::OrderPtr MissionBoard::current() const {
    return m_current;
}

des::OrderPtr MissionBoard::effective() const {
    if (auto interrupt = m_interrupt.active()) {
        return interrupt;
    }
    return m_current;
}

void MissionBoard::updateState(const des::MissionState& newState) {
    assert(m_current != nullptr);
    DES_LOG_DEBUG(rclcpp::get_logger("des.context.mission"), "Mission %d (type=%s) state: %s -> %s", m_current->id, m_current->type.c_str(), des::missionStateStr(m_current->state).c_str(), des::missionStateStr(newState).c_str());
    m_current->state = newState;
}

void MissionBoard::complete(ISimContext& ctx, const des::OrderPtr& order) {
    assert(order != nullptr);
    if (order->type != kChargeOrderType) {
        const int deadline = order->deadline.value_or(ctx.getTime());
        const int timeDiff = ctx.getTime() - deadline;

        const std::string detail = OrderRegistry::instance().get(order->type).outcomeDetail(*order);
        std::string summary = des::missionStateStr(order->state);
        if (!detail.empty()) {
            summary += " (" + detail + ")";
        }
        if (order->deadline.has_value()) {
            summary += std::format(" [{:+d}s]", timeDiff);
        }
        const bool routine = order->execution != des::ExecutionMode::SCHEDULED
            && order->state == des::MissionState::COMPLETED;
        if (routine) {
            DES_LOG_DEBUG(rclcpp::get_logger("des.mission"), "Mission %d (%s) %s", order->id, order->type.c_str(), summary.c_str());
        } else {
            DES_LOG_INFO(rclcpp::get_logger("des.mission"), "Mission %d (%s) %s", order->id, order->type.c_str(), summary.c_str());
        }

        m_bus.notifyMissionComplete(ctx.getTime(), order, timeDiff);
    }
    if (m_current == order) {
        setCurrent(nullptr);
    }
}

void MissionBoard::addScheduled(const des::OrderPtr& orderPtr) {
    m_scheduled.add(orderPtr);
}

bool MissionBoard::hasScheduled() const {
    return m_scheduled.has();
}

des::OrderPtr MissionBoard::nextScheduled() {
    return m_scheduled.peek();
}

des::OrderPtr MissionBoard::popScheduled() {
    return m_scheduled.pop();
}

std::optional<int> MissionBoard::nextScheduledDispatchTime() const {
    return m_queue.nextDispatchTime();
}

des::OrderPtr MissionBoard::peekNextScheduledOrder() const {
    auto event = m_queue.nextDispatchEvent();
    if (!event) {
        return nullptr;
    }
    auto dispatch = std::dynamic_pointer_cast<MissionDispatchEvent>(event);
    return dispatch ? dispatch->orderPtr : nullptr;
}

std::vector<des::OrderPtr> MissionBoard::peekScheduledOrdersUntil(const int untilTime) const {
    std::vector<des::OrderPtr> orders;
    for (const auto& event : m_queue.dispatchEventsUntil(untilTime)) {
        auto dispatch = std::dynamic_pointer_cast<MissionDispatchEvent>(event);
        if (dispatch && dispatch->orderPtr) {
            orders.push_back(dispatch->orderPtr);
        }
    }
    return orders;
}

void MissionBoard::addBackground(const des::OrderPtr& orderPtr) {
    m_background.add(orderPtr);
}

bool MissionBoard::hasBackground() const {
    return m_background.has();
}

des::OrderPtr MissionBoard::acceptFeasibleBackground(ISimContext& ctx) {
    if (!m_background.hasPlanned()) {
        m_background.plan(ctx);
    }
    auto accepted = m_background.popPlanned();
    if (accepted) {
        m_current = accepted;
    }
    return accepted;
}

bool MissionBoard::pushInterrupt(ISimContext& ctx, const des::OrderPtr& order) {
    assert(order->execution == des::ExecutionMode::INTERRUPT && "Interrupt pushed with wrong ExecutionMode");
    const auto robot = ctx.getRobot();
    if (robot->isBatteryLow()) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.context.interrupt"), "Reject %d (type=%s) — battery low (SoC %.0f%%), heading to dock", order->id, order->type.c_str(), robot->batteryStats().soc * 100.0);
        return false;
    }
    if (!m_interrupt.push(order, m_current)) {
        return false;
    }

    // Shifts the in-flight activity-end event by the interrupt's duration
    const bool wasDriving = robot->isDriving();

    auto& plugin = OrderRegistry::instance().get(order->type);
    const int duration = static_cast<int>(plugin.estimateMissionDuration(*order, ctx, robot->getLocation()));

    if (auto e = robot->inFlight().lock()) {
        const int oldTime = e->time;
        const int newTime = oldTime + duration;
        e->cancelled = true;
        auto shifted = e->withTime(newTime);
        ctx.startActivity(shifted);
        DES_LOG_DEBUG(rclcpp::get_logger("des.context.interrupt"), "Push %d (type=%s, dur=%ds) at t=%d — shifted in-flight '%s': %d → %d", order->id, order->type.c_str(), duration, ctx.getTime(), e->getName().c_str(), oldTime, newTime);
    } else {
        DES_LOG_DEBUG(rclcpp::get_logger("des.context.interrupt"), "Push %d (type=%s, dur=%ds) at t=%d — robot idle, no in-flight to shift", order->id, order->type.c_str(), duration, ctx.getTime());
    }

    if (wasDriving) {
        robot->setDriving(false);
        m_bus.notifyEvent(ctx.getTime(), des::EventType::STOP_DRIVE, "Drive paused: " + robot->getLocation(), false, robot->isCharging(), "", order->id);
    }

    m_interrupt.suspend(wasDriving);
    plugin.onMissionStart(ctx, *order);
    return true;
}

void MissionBoard::popInterrupt(ISimContext& ctx, const des::OrderPtr& completedOrder) {
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
        m_bus.notifyEvent(ctx.getTime(), des::EventType::START_DRIVE, "Drive resumed: " + robot->getTargetLocation(), true, robot->isCharging(), "", -1);
    }
    if (ctx.getConfig()->replanBackgroundOnInterrupt) {
        m_background.invalidatePlan();
    }
    DES_LOG_DEBUG(rclcpp::get_logger("des.context.interrupt"), "Pop %d (type=%s) at t=%d — resuming main mission", completedOrder->id, completedOrder->type.c_str(), ctx.getTime());
}

bool MissionBoard::hasActiveInterrupt() const {
    return m_interrupt.has();
}

void MissionBoard::reset() {
    DES_LOG_DEBUG(rclcpp::get_logger("des.context.mission"), "Reset (pending=%zu, background=%zu, interrupt=%s cleared)", m_scheduled.size(), m_background.size(), m_interrupt.has() ? "yes" : "no");
    m_current = nullptr;
    m_scheduled.clear();
    m_background.clear();
    m_interrupt.clear();
}
