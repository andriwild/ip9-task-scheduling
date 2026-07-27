#include "sim_runner.h"
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include "../../util/log.h"
#include <thread>
#include <utility>

#include "../../behaviour/bt_setup.h"
#include "../../observer/ros.h"
#include "event_system_msgs/srv/set_system_state.hpp"


void SimRunner::reloadSimulationData() {
    m_orders = loadOrders(m_config->appointmentsPath, m_config->simStartTime, m_config->simStartTime + m_config->simDuration);
    m_backgroundTemplates = ConfigLoader::loadBackgroundTemplates(m_config->appointmentsPath);
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Successful loaded %zu background templates", m_backgroundTemplates.size());
    auto allPeople = ConfigLoader::loadEmployees(m_config->employeesPath);
    if (!allPeople.has_value() || allPeople.value().empty()) {
        throw std::runtime_error("No employees loaded");
    }

    ConfigLoader::validateConfig(m_orders, allPeople.value(), m_locationMap, "5.2B_Elevator");

    m_people = std::move(allPeople.value());
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Simulating %zu employees", m_people.value().size());

    m_employees.clear();
    for (const auto& p: m_people.value()) {
        m_employees[p->firstName] = p.get();
    }
}

void SimRunner::rebuildEventQueue() {
    populateEventQueue();
}

void SimRunner::buildSimulation() {
    m_roomTours = loadRoomTours();
    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue, m_config, m_planner, m_employees, m_locationMap, m_roomTours
    );
    m_ctx->addObserver(m_metricsNode);
    m_ctx->addObserver(m_rosObserver);
    m_personMarkerObserver->attach(m_ctx.get(), m_employees);
    m_ctx->addObserver(m_personMarkerObserver);
    m_ctx->addObserver(m_robotMarkerObserver);
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

    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "System Reset Complete");
}

void SimRunner::setupApplication() {
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Setup Application...");

    m_config = m_systemConfigNode->getConfig();

    constexpr int kMaxInteractiveDurationSec = 3 * 86400;
    if (m_config->simDuration > kMaxInteractiveDurationSec) {
        throw std::runtime_error(
            "Interactive mode is limited to 3 days of simulation, but sim_duration=" +
            std::to_string(m_config->simDuration) + "s (" +
            std::to_string(m_config->simDuration / 86400) + " days). Reduce sim_duration to <= " +
            std::to_string(kMaxInteractiveDurationSec) + "s or run headless.");
    }

    reloadSimulationData();
    m_rosObserver = std::make_shared<RosObserver>(m_systemConfigNode);
    m_personMarkerObserver = std::make_shared<PersonMarkerObserver>(m_systemConfigNode, m_locationMap);
    m_robotMarkerObserver = std::make_shared<RobotMarkerObserver>(m_systemConfigNode, m_locationMap);
    buildSimulation();

    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Setup Complete!");
}

void SimRunner::updateConfig(std::shared_ptr<des::SimConfig> config) {
    m_config = std::move(config);
    m_ctx->setConfig(m_config);
    DES_LOG_DEBUG_STREAM(rclcpp::get_logger("des.runner"), *m_config.get());
}

void SimRunner::updateConfig() {
    if (m_systemConfigNode->isConfigDirty()) {
        updateConfig(m_systemConfigNode->getConfig());
        m_systemConfigNode->clearDirty();
    }
}

void SimRunner::enterPause() const {
    m_controllerNode->currentState.store(SystemState::Request::PAUSE);
    DES_LOG_DEBUG(rclcpp::get_logger("des.runner"), "Simulation loop paused");
}

int SimRunner::loadAppState() const {
    return m_controllerNode->currentState.load();
}
