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
#include "../../model/event_queue.h"

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
    m_allPeople = allPeople.value();

    // employees needs all people for scheduler path lookups
    for (const auto& p: m_allPeople) {
        m_employees[p->firstName] = p;
    }

    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue,
        m_config,
        m_planner,
        m_employees,
        m_locationMap
    );

    rebuildFileQueue();

    if (m_config->metricsCsvExport) {
        const std::time_t now = std::time(nullptr);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
        m_metricsNode->enableCsv(CONFIG_PATH + "../results/metrics_" + stamp + ".csv", m_config);
    }
    m_ctx->addObserver(m_metricsNode);

    if (m_config->missionTraceExport) {
        const std::time_t traceNow = std::time(nullptr);
        char traceStamp[32];
        std::strftime(traceStamp, sizeof(traceStamp), "%Y%m%d_%H%M%S", std::localtime(&traceNow));
        const std::string tracePath = CONFIG_PATH + "../results/mission_trace_" + traceStamp + ".json";
        m_ctx->addObserver(std::make_shared<MissionTraceObserver>(m_ctx.get(), m_locationMap, tracePath));
    }

    m_ctx->setBehaviorTree(setupBehaviorTree(m_ctx));

    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Rounds: %d, scenario: %s",
                m_totalRounds, m_config->appointmentsPath.c_str());

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

    auto appts = ConfigLoader::loadOrderConfig(path, m_config->simStartTime, m_config->simStartTime + m_config->simDuration);
    if (!appts.has_value()) {
        DES_LOG_ERROR(rclcpp::get_logger("des.runner"), "Failed to load appointments from: %s", path.c_str());
        return loadNextBatch();
    }
    m_orders = appts.value();

    m_backgroundTemplates = ConfigLoader::loadBackgroundTemplates(path);
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Successful loaded %zu background templates", m_backgroundTemplates.size());

    ConfigLoader::validateConfig(m_orders, m_allPeople, m_locationMap, "5.2B_Elevator");

    m_people = ConfigLoader::filterByAppointments(m_allPeople, m_orders);
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Simulating %zu of %zu employees",
                m_people.value().size(), m_allPeople.size());

    populateEventQueue();
    m_eventQueue.print();
    return true;
}

namespace {
void logSightingSummary(const Robot& robot) {
    const auto sightings = robot.getSightings();
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot.sightings"), "Sightings recorded: %zu", sightings.size());

    std::map<std::string, std::map<std::string, std::pair<int, int>>> counts;
    for (const auto& s : sightings) {
        auto& [present, absent] = counts[s.personName][s.location];
        if (s.kind == SightingKind::PRESENT) {
            present++;
        } else {
            absent++;
        }
    }

    for (const auto& [person, rooms] : counts) {
        std::vector<std::tuple<double, std::string, int, int>> rows;
        for (const auto& [room, presentAbsent] : rooms) {
            const auto& [present, absent] = presentAbsent;
            const double rate = present + absent > 0 ? static_cast<double>(present) / (present + absent) : 0.0;
            rows.emplace_back(rate, room, present, absent);
        }
        std::sort(rows.begin(), rows.end(), std::greater<>());
        for (const auto& [rate, room, present, absent] : rows) {
            DES_LOG_DEBUG(rclcpp::get_logger("des.robot.sightings"), "%-8s %-18s present=%4d absent=%4d rate=%.3f",
                         person.c_str(), room.c_str(), present, absent, rate);
        }
    }
}
}

void HeadlessRunner::onSimulationComplete() {
    logSightingSummary(*m_ctx->getRobot());

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
