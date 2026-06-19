#include "headless_runner.h"
#include "../../util/log.h"
#include <ctime>
#include <memory>
#include <filesystem>
#include "../../behaviour/bt_setup.h"
#include "event_system_msgs/srv/detail/set_system_state__struct.hpp"
#include "../../model/event_queue.h"

void HeadlessRunner::setupApplication(const std::string& path) {
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Setup Application...");

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
        m_planner,
        m_employeeLocations,
        m_locationMap
    );

    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        m_orderFilePaths.push_back(entry.path());
    }
    std::sort(m_orderFilePaths.begin(), m_orderFilePaths.end());
    if (m_orderFilePaths.empty()) {
        throw std::runtime_error("No order files found in: " + path);
    }
    rebuildFileQueue();

    if (m_config->metricsCsvExport) {
        const std::time_t now = std::time(nullptr);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
        m_metricsNode->enableCsv(CONFIG_PATH + "../results/metrics_" + stamp + ".csv", m_config);
    }
    m_ctx->addObserver(m_metricsNode);
    m_ctx->setBehaviorTree(setupBehaviorTree(m_ctx));

    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Rounds: %d, Files per round: %zu",
                m_totalRounds, m_orderFilePaths.size());

    loadNextBatch();

    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Setup Complete!");
}

bool HeadlessRunner::loadNextBatch() {
    if (m_orderFiles.empty()) {
        m_currentRound++;
        if (m_currentRound >= m_totalRounds) {
            return false;
        }
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Starting round %d/%d",
                    m_currentRound + 1, m_totalRounds);
        rebuildFileQueue();
    }

    while (!m_eventQueue.empty()) {
        m_eventQueue.pop();
    }

    auto path = m_orderFiles.front();
    m_orderFiles.pop();
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Loading batch: %s (round %d/%d)",
                path.c_str(), m_currentRound + 1, m_totalRounds);
    m_metricsNode->setRunInfo(std::filesystem::path(path).stem().string(), m_currentRound + 1);

    auto appts = ConfigLoader::loadOrderConfig(path);
    if (!appts.has_value()) {
        DES_LOG_ERROR(rclcpp::get_logger("des.runner"), "Failed to load appointments from: %s", path.c_str());
        return loadNextBatch();
    }
    m_orders = appts.value();

    m_backgroundOrders = ConfigLoader::loadBackgroundOrders(path);
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Successful loaded %zu background orders", m_backgroundOrders.size());

    ConfigLoader::validateConfig(m_orders, m_allPeople, m_locationMap, "5.2B_Elevator");

    m_people = ConfigLoader::filterByAppointments(m_allPeople, m_orders);
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Simulating %zu of %zu employees",
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
    // not needed in headless mode
}

void HeadlessRunner::enterPause() const {
    // not needed in headless mode
}

void HeadlessRunner::updateConfig() {
    // not needed in headless mode
}
