#include "headless_runner.h"
#include <memory>
#include <filesystem>
#include "../../behaviour/bt_setup.h"
#include "event_system_msgs/srv/detail/set_system_state__struct.hpp"
#include "../../model/event_queue.h"

void HeadlessRunner::setupApplication(const std::string& path) {
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Setup Application...");

    if (!std::filesystem::exists(path) || !std::filesystem::is_directory(path)) {
        throw std::runtime_error("Appointment path does not exist or is not a directory: " + path);
    }

    m_config = std::make_shared<des::SimConfig>(ConfigLoader::loadSimConfig().value());
    auto allPeople = ConfigLoader::loadEmployees(CONFIG_PATH + "employee.json");
    if (!allPeople.has_value() || allPeople.value().empty()) {
        throw std::runtime_error("No employees loaded");
    }
    m_allPeople = allPeople.value();

    // employeeLocations needs all people for scheduler path lookups
    for (const auto& p: m_allPeople) {
        m_employeeLocations[p->firstName] = p;
    }

    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue,
        m_config,
        m_plannerNode,
        m_employeeLocations,
        m_searchAreaMap
    );

    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        m_orderFilePaths.push_back(entry.path());
    }
    std::sort(m_orderFilePaths.begin(), m_orderFilePaths.end());
    rebuildFileQueue();

    m_ctx->addObserver(m_metricsNode);
    m_ctx->setBehaviorTree(setupBehaviorTree(m_ctx));

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Rounds: %d, Files per round: %zu",
                m_totalRounds, m_orderFilePaths.size());

    loadNextBatch();

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Setup Complete!");
}

bool HeadlessRunner::loadNextBatch() {
    if (m_orderFiles.empty()) {
        m_currentRound++;
        if (m_currentRound >= m_totalRounds) {
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Starting round %d/%d",
                    m_currentRound + 1, m_totalRounds);
        rebuildFileQueue();
    }

    while (!m_eventQueue.empty()) {
        m_eventQueue.pop();
    }

    auto path = m_orderFiles.front();
    m_orderFiles.pop();
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Loading batch: %s (round %d/%d)",
                path.c_str(), m_currentRound + 1, m_totalRounds);

    auto appts = ConfigLoader::loadOrderConfig(path);
    if (!appts.has_value()) {
        RCLCPP_ERROR(rclcpp::get_logger("des_application"), "Failed to load appointments from: %s", path.c_str());
        return loadNextBatch();
    }
    m_orders = appts.value();

    ConfigLoader::validateConfig(m_orders, m_allPeople, m_locationMap, "5.2B_Elevator");

    m_people = ConfigLoader::filterByAppointments(m_allPeople, m_orders);
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Simulating %zu of %zu employees",
                m_people.value().size(), m_allPeople.size());

    populateEventQueue();
    m_eventQueue.print();
    return true;
}

void HeadlessRunner::onSimulationComplete() {
    m_metricsNode->publishReport();
    m_metricsNode->clear();

    if (!loadNextBatch()) {
        m_batchComplete = true;
    }
}

int HeadlessRunner::loadAppState() const {
    if (m_batchComplete) {
        return event_system_msgs::srv::SetSystemState::Request::EXIT;
    }
    return event_system_msgs::srv::SetSystemState::Request::RUN;
}

void HeadlessRunner::reset() {
    // not needed — batch sequencing handled by loadNextBatch()
}

void HeadlessRunner::enterPause() const {
    // not needed in headless mode
}

void HeadlessRunner::updateConfig() {
    // not needed in headless mode
}
