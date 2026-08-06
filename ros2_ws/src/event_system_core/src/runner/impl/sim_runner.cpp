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
}

void SimRunner::rebuildEventQueue() {
    populateEventQueue();
}

void SimRunner::buildSimulation() {
    mergeRoomTours();

    auto allPeople = ConfigLoader::loadEmployees(m_config->employeesPath);
    if (!allPeople.has_value() || allPeople.value().empty()) {
        throw std::runtime_error("No employees loaded");
    }
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Simulating %zu employees", allPeople.value().size());

    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue, m_config, m_planner, std::move(allPeople.value()), m_rooms
    );
    m_ctx->addObserver(m_rosObserver);
    rebuildEventQueue();
    m_ctx->setBehaviorTree(setupBehaviorTree(m_ctx.get()));
}


void SimRunner::reset() {
    while (!m_eventQueue.empty()) {
        m_eventQueue.pop();
    }

    m_rosObserver->publishReset();
    m_protocol.clear();

    m_ctx.reset();
    reloadSimulationData();
    buildSimulation();

    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "System Reset Complete");
}

void SimRunner::setupApplication() {
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Setup Application...");

    m_config = m_systemConfigNode->getConfig();

    // simulation with duration > 3 days leads to message overflow
    constexpr int kMaxInteractiveDurationSec = 3 * SECONDS_PER_DAY;
    if (m_config->simDuration > kMaxInteractiveDurationSec) {
        throw std::runtime_error(
            "Interactive mode is limited to 3 days of simulation, but sim_duration=" +
            std::to_string(m_config->simDuration) + "s (" +
            std::to_string(m_config->simDuration / SECONDS_PER_DAY) + " days). Reduce sim_duration to <= " +
            std::to_string(kMaxInteractiveDurationSec) + "s or run headless.");
    }

    reloadSimulationData();
    m_rosObserver = std::make_shared<RosObserver>(m_systemConfigNode);
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
