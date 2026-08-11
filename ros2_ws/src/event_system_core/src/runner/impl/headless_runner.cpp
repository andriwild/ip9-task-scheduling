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
#include "engine/event_queue.h"

namespace des {

void HeadlessRunner::setupApplication() {
    DES_LOG_INFO("des.runner", "Setup Application...");

    m_config = std::make_shared<SimConfig>(ConfigLoader::loadSimConfig().value());

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

    if (m_config->metricsCsvExport) {
        m_reporter.enableCsv(outputPath("metrics", ".csv"));
        m_reporter.enableDailyCsv(outputPath("metrics_daily", ".csv"));
    }
    if (m_config->debugExport) {
        m_reporter.enableDebugTrace(outputPath("debug", "/"));
    }

    m_ctx->setBehaviorTree(setupBehaviorTree(m_ctx.get()));

    loadNextRound();

    DES_LOG_INFO("des.runner", "Setup Complete!");
}

bool HeadlessRunner::loadNextRound() {
    if (m_currentRound >= m_config->rounds) {
        return false;
    }

    while (!m_eventQueue.empty()) {
        m_eventQueue.pop();
    }
    m_protocol.clear();

    if (m_config->roundMode == RoundMode::REPLICATION) {
        m_ctx->reseed(roundSeed(m_currentRound));
    }

    const std::string& path = m_config->appointmentsPath;
    DES_LOG_INFO("des.runner", "Starting round %d/%d: %s", m_currentRound + 1, m_config->rounds, path.c_str());
    m_reporter.setRunInfo(std::filesystem::path(path).stem().string(), m_ctx->activeSeed());

    auto appts = ConfigLoader::loadOrderConfig(path, m_config->simStartTime, m_config->simStartTime + m_config->simDuration);
    if (!appts.has_value()) {
        DES_LOG_ERROR("des.runner", "Failed to load appointments from: %s", path.c_str());
        return false;
    }
    m_orders = appts.value();

    m_backgroundTemplates = ConfigLoader::loadBackgroundTemplates(path);
    DES_LOG_INFO("des.runner", "Successful loaded %zu background templates", m_backgroundTemplates.size());

    populateEventQueue();
    m_eventQueue.print();
    m_currentRound++;
    return true;
}

void HeadlessRunner::onSimulationComplete() {
    if (!loadNextRound()) {
        m_runComplete = true;
    }
}

RunState HeadlessRunner::loadAppState() const {
    return m_runComplete ? RunState::Exit : RunState::Run;
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

}  // namespace des
