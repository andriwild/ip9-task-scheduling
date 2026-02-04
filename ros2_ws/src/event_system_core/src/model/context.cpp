#include "context.h"
#include "../util/rnd.h"

SimulationContext::SimulationContext(
    EventQueue& queue,
    std::shared_ptr<des::SimConfig> simConfig,
    std::shared_ptr<PathPlannerNode> plannerNode,
    std::map<std::string, std::vector<std::string>> employeeLocations,
    rclcpp::Logger logger
) : m_simConfig(simConfig),
    m_logger(logger),
    m_plannerNode(plannerNode),
    m_queue(queue),
    m_employeeLocations(employeeLocations)
{
    m_robot = std::make_unique<Robot>(m_simConfig, logger);
    RCLCPP_INFO(m_logger, "Simulation Context created!");
}

void SimulationContext::scheduleArrival(int currentTime, const std::string target, bool isMissionComplete) {
    int arrivalTime = currentTime + 1;

    if (m_robot->getLocation() == target) {
        this->m_queue.push(std::make_shared<ArrivedEvent>(currentTime, target, 0));
        RCLCPP_INFO(m_logger, "Already at %s", target.c_str());
    } else {
        std::optional<double> distance = this->m_plannerNode->calcDistance(
            this->m_robot->getLocation(), 
            target,
            m_simConfig->cacheEnabled
        );

        assert(distance.has_value());
        assert(this->m_robot->getCurrentSpeed() > 0);

        double travelTime = distance.value() / this->m_robot->getCurrentSpeed();

        double noiseFactor = rnd::getNormalDist(1.0, 0.1);
        double timeVariance = travelTime * noiseFactor;
        arrivalTime = currentTime + timeVariance;

        this->m_queue.push(std::make_shared<ArrivedEvent>(arrivalTime, target, distance.value()));
        RCLCPP_INFO(m_logger, "Moving to %s (%.1fs + %.1f)", 
                    target.c_str(),
                    travelTime, 
                    timeVariance - travelTime);
    }

    if (isMissionComplete) {
        this->m_queue.push(std::make_shared<MissionCompleteEvent>(arrivalTime));
    }
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
    m_robot->setDriveSpeed(m_simConfig->robotSpeed);
    m_robot->setAccompanytSpeed(m_simConfig->robotAccompanySpeed);
    RCLCPP_INFO(rclcpp::get_logger("SimulationContext"), "New config active");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("SimulationContext"), *m_simConfig);
}

void SimulationContext::changeRobotState(std::unique_ptr<RobotState> newState) {
    // get the energy consumption of the previous state
    m_robot->m_bat->updateBalance(m_currentTime, m_robot->getState()->getEnergyConsumption(*this));
    m_robot->changeState(std::move(newState));
    notifyRobotStateChanged(m_robot->getState()->getType());
    notifyBatteryState(m_currentTime, m_robot->m_bat->getStats().soc, m_robot->m_bat->getStats().capacity);
}
