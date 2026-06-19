#include "context.h"
#include "../util/log.h"

#include <utility>
#include "../util/rnd.h"
#include "../sim/scheduler.h"
#include "event/mission_dispatch_event.h"
#include "../plugins/charge/charge_order.h"

SimulationContext::SimulationContext(
    EventQueue& queue,
    std::shared_ptr<des::SimConfig> simConfig,
    std::shared_ptr<IPathPlanner> plannerNode,
    std::map<std::string, std::shared_ptr<des::Person>> employeeLocations,
    des::LocationMap locationMap
)
    : m_simConfig(std::move(simConfig))
    , m_queue(queue)
    , m_employeeLocations(std::move(employeeLocations))
    , m_scheduler(std::make_unique<Scheduler>(m_simConfig, plannerNode, m_employeeLocations, m_locationMap))
    , m_plannerNode(std::move(plannerNode))
    , m_locationMap(std::move(locationMap))
{
    m_robot = std::make_unique<Robot>(m_simConfig);
    DES_LOG_INFO(rclcpp::get_logger("des.context"), "Simulation Context created!");
}

Journey SimulationContext::scheduleArrival(const std::string& target) const {
    const std::optional<double> distance = this->m_plannerNode->calcDistance(
        this->m_robot->getLocation(), 
        target,
        m_simConfig->cacheEnabled
    );
    assert(distance.has_value());

    const double travelTime = distance.value() / this->m_robot->getCurrentSpeed();
    double travelTimeRnd = rnd::normal(m_rng, travelTime, getDriveTimeStd());
    if (travelTimeRnd < 0) {
        travelTimeRnd = travelTime;
    }
    return { travelTimeRnd, distance.value() };
}

void SimulationContext::completeOrder(const des::OrderPtr& order) {
    assert(order != nullptr);
    if (order->type != kChargeOrderType) {
        const int deadline = order->deadline.value_or(m_currentTime);
        const int timeDiff = m_currentTime - deadline;
        notifyMissionComplete(order->state, timeDiff, order->execution);
    }

    if (m_currentMission == order) {
        setOrderPtr(nullptr);
    }
}

void SimulationContext::resetContext(const int newTime) {
    m_currentTime = newTime;
    des::log::setSimTime(newTime);
    DES_LOG_INFO(rclcpp::get_logger("des.context.mission"), "Reset (pending=%zu, background=%zu, interrupt=%s cleared)", m_scheduledMissions.size(), m_backgroundMissions.size(), m_interruptMission.has() ? "yes" : "no");
    m_currentMission = nullptr;
    m_scheduledMissions.clear();
    m_backgroundMissions.clear();
    m_interruptMission.clear();
    m_personLocations.clear();
    resetRobot();
}

des::OrderPtr SimulationContext::peekNextScheduledOrder() const {
    auto event = m_queue.nextDispatchEvent();
    if (!event) return nullptr;
    auto dispatch = std::dynamic_pointer_cast<MissionDispatchEvent>(event);
    return dispatch ? dispatch->orderPtr : nullptr;
}

void SimulationContext::setConfig(const std::shared_ptr<des::SimConfig> &newConfig) {
    m_simConfig = newConfig;
    m_robot->updateConfig(*newConfig);
    DES_LOG_INFO_STREAM(rclcpp::get_logger("des.context"), *m_simConfig);
}

void SimulationContext::changeRobotState(std::unique_ptr<RobotState> newState) const {
    const auto* current = m_robot->getState();
    const auto newType = newState->getType();
    const bool sameState = current
        && current->getType() == newType
        && current->getName() == newState->getName();
    const bool leavingOpportunisticCharge = current
        && current->getType() == des::RobotStateType::CHARGING
        && newType != des::RobotStateType::CHARGING
        && m_robot->m_opportunisticCharge;
    m_robot->changeState(std::move(newState));
    if (leavingOpportunisticCharge) {
        m_queue.cancelByType(des::EventType::BATTERY_FULL);
        m_queue.cancelByType(des::EventType::CHARGE_PHASE_TRANSITION);
        m_robot->m_batteryFullEventScheduled = false;
        m_robot->m_opportunisticCharge = false;
    }
    if (!sameState) {
        notifyRobotStateChanged();
    }
}
