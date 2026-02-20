#include "context.h"

#include <utility>
#include "../util/rnd.h"
#include "event.h"

SimulationContext::SimulationContext(
    EventQueue& queue,
    std::shared_ptr<des::SimConfig> simConfig,
    std::shared_ptr<PathPlannerNode> plannerNode,
    std::map<std::string, std::vector<std::string>> employeeLocations,
    Scheduler& scheduler
)
    : m_simConfig(std::move(simConfig))
    , m_scheduler(scheduler)
    , m_plannerNode(std::move(plannerNode))
    , m_queue(queue)
    , m_employeeLocations(std::move(employeeLocations))
{
    m_robot = std::make_unique<Robot>(m_simConfig);
    RCLCPP_INFO(rclcpp::get_logger("Context"), "Simulation Context created!");
}

Journey SimulationContext::scheduleArrival(const std::string& target) const {
    const std::optional<double> distance = this->m_plannerNode->calcDistance(
        this->m_robot->getLocation(), 
        target,
        m_simConfig->cacheEnabled
    );
    assert(distance.has_value());

    const double travelTime = distance.value() / this->m_robot->getCurrentSpeed();
    const double noiseFactor = rnd::getNormalDist(1.0, 0.1);
    return { travelTime * noiseFactor, distance.value() };
}

void SimulationContext::updateAppointmentState(const des::MissionState& newState) const {
    assert(m_currentAppointment != nullptr);
    m_currentAppointment->state = newState;
}

void SimulationContext::completeAppointment(const std::shared_ptr<des::Appointment>& appt) const {
    assert(appt != nullptr);
    const int timeDiff = m_currentTime - appt->appointmentTime;
    notifyMissionComplete(appt->state, timeDiff);
}

void SimulationContext::resetContext(const int newTime) {
    m_currentTime = newTime;
    m_robot->setLocation(m_robot->getIdleLocation());
    m_robot->m_bat->reset(newTime);
    m_pendingMissions = std::queue<std::shared_ptr<des::Appointment>>();
}

double SimulationContext::getRndConversationTime() const {
    return rnd::getNormalDist(
        getDefaultConversationTime(),
        getConversationDurationStd()
    );
}

void SimulationContext::setConfig(const std::shared_ptr<des::SimConfig> &newConfig) {
    m_simConfig = newConfig;
    m_robot->updateConfig(*newConfig);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Context"), *m_simConfig);
}

void SimulationContext::changeRobotState(std::unique_ptr<RobotState> newState) const {
    // get the energy consumption of the previous state
    m_robot->m_bat->updateBalance(m_currentTime, m_robot->getState()->getEnergyConsumption(*this));
    if (m_robot->m_bat->isBatteryLow()) {
        m_robot->setChargingRequired(true);
    }
    m_robot->changeState(std::move(newState));
    notifyRobotStateChanged(m_robot->getState()->getType());
}