#include "sim_runner.h"
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <utility>

#include "../../behaviour/bt_setup.h"
#include "../../observer/ros.h"
#include "event_system_msgs/srv/set_system_state.hpp"


void SimRunner::reloadSimulationData() {
    m_appointments = loadAppointments(m_config->appointmentsPath);
    auto allPeople = ConfigLoader::loadEmployees(CONFIG_PATH + "employee.json");
    if (!allPeople.has_value() || allPeople.value().empty()) {
        throw std::runtime_error("No employees loaded");
    }

    ConfigLoader::validateConfig(m_appointments, allPeople.value(), m_locationMap, "5.2B_Elevator");

    m_people = ConfigLoader::filterByAppointments(allPeople.value(), m_appointments);
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Simulating %zu of %zu employees", m_people.value().size(), allPeople.value().size());

    m_employeeLocations.clear();
    for (const auto& p: m_people.value()) {
        m_employeeLocations[p->firstName] = p;
    }
}

void SimRunner::rebuildEventQueue() {
    if (!m_ctx) {
        throw std::runtime_error("rebuildEventQueue requires initialized SimulationContext");
    }

    scheduleOccupancy(*m_config, m_people.value(), m_ctx->m_rng);
    m_eventQueue.extend(IAppRunner::personArrivalGenerator(m_people.value()));
    m_eventQueue.extend(IAppRunner::createMissionQueue(m_config, m_appointments, m_ctx->getScheduler(), "IMVS_Dock"));

    int firstEventTime = m_eventQueue.getFirstEventTime() - ONE_HOUR;
    int lastEventTime  = m_eventQueue.getLastEventTime();

    auto latest = std::max_element(m_people.value().begin(), m_people.value().end(),
        [](const auto& a, const auto& b) { return a->departureTime < b->departureTime; });
    int lastDepartureTime = (*latest)->departureTime;

    RCLCPP_DEBUG(rclcpp::get_logger("des_application"), "Event time range from %d to %d", firstEventTime, lastEventTime);

    int simEndTime = std::max(lastEventTime, lastDepartureTime) + ONE_HOUR;
    m_eventQueue.push(std::make_shared<SimulationStartEvent>(firstEventTime));
    m_eventQueue.push(std::make_shared<SimulationEndEvent>(simEndTime));

    for (auto& p : m_people.value()) {
        m_eventQueue.push(std::make_shared<PersonTransitionEvent>(firstEventTime, p));
        m_eventQueue.push(std::make_shared<PersonTransitionEvent>(simEndTime, p));
    }

    m_ctx->resetContext(m_eventQueue.getFirstEventTime());

    for (auto& p : m_people.value()) {
        m_ctx->setPersonLocation(p->firstName, "OUTDOOR");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    publishMissions(m_eventQueue, m_rosObserver);
}

void SimRunner::buildSimulation() {
    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue, m_config, m_plannerNode, m_employeeLocations
    );
    m_ctx->addObserver(m_metricsNode);
    m_ctx->addObserver(m_rosObserver);
    rebuildEventQueue();
    m_ctx->setBehaviorTree(setupBehaviorTree(m_ctx));
}


void SimRunner::reset() {
    while (!m_eventQueue.empty()) {
        m_eventQueue.pop();
    }

    m_rosObserver->publishReset();
    m_metricsNode->clear();

    reloadSimulationData();
    buildSimulation();

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "System Reset Complete");
}


void SimRunner::setupApplication(const std::string& /*path*/) {
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Setup Application...");

    m_config = m_systemConfigNode->getConfig();
    reloadSimulationData();
    m_rosObserver = std::make_shared<RosObserver>(m_systemConfigNode);
    buildSimulation();

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Setup Complete!");
}

void SimRunner::updateConfig(std::shared_ptr<des::SimConfig> config) {
    m_config = std::move(config);
    m_ctx->setConfig(m_config);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("des_application"), *m_config.get());
}


void SimRunner::updateConfig() {
    if (m_systemConfigNode->isConfigDirty()) {
        updateConfig(m_systemConfigNode->getConfig());
        m_systemConfigNode->clearDirty();
    }
}

void SimRunner::enterPause() const {
    m_controllerNode->currentState.store(SystemState::Request::PAUSE);
    RCLCPP_DEBUG(rclcpp::get_logger("des_application"), "Simulation loop paused");
}

int SimRunner::loadAppState() const {
    const auto state = m_controllerNode->currentState.load();
    return state;
}
