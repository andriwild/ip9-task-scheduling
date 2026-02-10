#include "context.h"
#include "../util/rnd.h"
#include "event.h"

SimulationContext::SimulationContext(
    EventQueue& queue,
    std::shared_ptr<des::SimConfig> simConfig,
    std::shared_ptr<PathPlannerNode> plannerNode,
    std::map<std::string, std::vector<std::string>> employeeLocations,
    rclcpp::Logger logger
)
    : m_simConfig(simConfig)
    , m_logger(logger)
    , m_plannerNode(plannerNode)
    , m_queue(queue)
    , m_employeeLocations(employeeLocations)
{
    m_robot = std::make_unique<Robot>(m_simConfig, logger);
    RCLCPP_INFO(m_logger, "Simulation Context created!");
}

Journey SimulationContext::scheduleArrival(const std::string target) {
    std::optional<double> distance = this->m_plannerNode->calcDistance(
        this->m_robot->getLocation(), 
        target,
        m_simConfig->cacheEnabled
    );
    assert(distance.has_value());

    double travelTime = distance.value() / this->m_robot->getCurrentSpeed();
    double noiseFactor = rnd::getNormalDist(1.0, 0.1);

    return { travelTime * noiseFactor, distance.value() };
}

void SimulationContext::updateAppointmentState(const des::MissionState& newState) {
    assert(m_currentAppointment != nullptr);
    m_currentAppointment->state = newState;
}

void SimulationContext::completeAppointment() {
    assert(m_currentAppointment != nullptr);
    int timeDiff = m_currentTime - m_currentAppointment->appointmentTime;
    notifyMissionComplete(m_currentAppointment->state, timeDiff);
}

void SimulationContext::resetContext(int newTime) {
    m_currentTime = newTime;
    m_robot->setLocation(m_robot->getIdleLocation());
    m_robot->m_bat->reset(newTime);
}

double SimulationContext::getRndConversationTime() {
    return rnd::getNormalDist(
        getDefaultConversationTime(),
        getConversationDurationStd()
    );
};

void SimulationContext::setConfig(std::shared_ptr<des::SimConfig> newConfig) {
    m_simConfig = newConfig;
    m_robot->updateConfig(*newConfig);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("SimulationContext"), *m_simConfig);
}

void SimulationContext::changeRobotState(std::unique_ptr<RobotState> newState) {
    // get the energy consumption of the previous state
    m_robot->m_bat->updateBalance(m_currentTime, m_robot->getState()->getEnergyConsumption(*this));
    m_robot->changeState(std::move(newState));
    notifyRobotStateChanged(m_robot->getState()->getType());
}
