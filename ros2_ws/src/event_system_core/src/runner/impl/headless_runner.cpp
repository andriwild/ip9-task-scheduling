#include "headless_runner.h"
#include "../../util/log.h"
#include <algorithm>
#include <ctime>
#include <map>
#include <memory>
#include <filesystem>
#include <tuple>
#include <vector>
#include "../../behaviour/bt_setup.h"
#include "../../observer/mission_trace.h"
#include "event_system_msgs/srv/detail/set_system_state__struct.hpp"
#include "engine/event_queue.h"

void HeadlessRunner::setupApplication() {
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Setup Application...");

    m_config = std::make_shared<des::SimConfig>(ConfigLoader::loadSimConfig().value());

    if (!std::filesystem::is_regular_file(m_config->appointmentsPath)) {
        throw std::runtime_error("Appointments file does not exist: " + m_config->appointmentsPath);
    }
    auto allPeople = ConfigLoader::loadEmployees(m_config->employeesPath);
    if (!allPeople.has_value() || allPeople.value().empty()) {
        throw std::runtime_error("No employees loaded");
    }
    mergeRoomTours();

    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue,
        m_config,
        m_planner,
        std::move(allPeople.value()),
        m_rooms
    );

    rebuildFileQueue();

    if (m_config->metricsCsvExport) {
        m_metricsNode->setRunId(s_runId);
        m_metricsNode->enableCsv(outputPath("metrics", ".csv"), m_config);
        m_metricsNode->enableDailyCsv(outputPath("metrics_daily", ".csv"));
        writeEffectiveConfig(outputPath("config", ".json"));
    }
    m_ctx->addObserver(m_metricsNode);

    if (m_config->missionTraceExport) {
        m_ctx->addObserver(std::make_shared<MissionTraceObserver>(m_ctx.get(), m_rooms, outputPath("mission_trace", ".json"), m_config));
    }

    m_ctx->setBehaviorTree(setupBehaviorTree(m_ctx.get()));

    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Rounds: %d, scenario: %s", m_totalRounds, m_config->appointmentsPath.c_str());

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

    if (m_config->roundMode == des::RoundMode::REPLICATION) {
        m_ctx->reseed(roundSeed(m_currentRound));
    }

    auto path = m_orderFiles.front();
    m_orderFiles.pop();
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Loading batch: %s (round %d/%d)", path.c_str(), m_currentRound + 1, m_totalRounds);
    m_metricsNode->setRunInfo(std::filesystem::path(path).stem().string(), m_currentRound + 1, m_ctx->activeSeed());

    auto appts = ConfigLoader::loadOrderConfig(path, m_config->simStartTime, m_config->simStartTime + m_config->simDuration);
    if (!appts.has_value()) {
        DES_LOG_ERROR(rclcpp::get_logger("des.runner"), "Failed to load appointments from: %s", path.c_str());
        return loadNextBatch();
    }
    m_orders = appts.value();

    m_backgroundTemplates = ConfigLoader::loadBackgroundTemplates(path);
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Successful loaded %zu background templates", m_backgroundTemplates.size());

    populateEventQueue();
    m_eventQueue.print();
    return true;
}

void HeadlessRunner::onSimulationComplete() {
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
