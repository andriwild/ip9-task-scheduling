#include "headless_runner.h"
#include <memory>
#include <filesystem>
#include "../../behaviour/bt_setup.h"
#include "event_system_msgs/srv/detail/set_system_state__struct.hpp"
#include "../../model/event_queue.h"

void HeadlessRunner::setupApplication(const std::string& path) {
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Setup Application...");

    assert(std::filesystem::exists(path) && std::filesystem::is_directory(path));

    m_config = std::make_shared<des::SimConfig>(ConfigLoader::loadSimConfig().value());
    auto allPeople = ConfigLoader::loadEmployees(CONFIG_PATH + "employee.json");
    assert(!allPeople.value().empty());
    m_allPeople = allPeople.value();

    // employeeLocations needs all people for scheduler path lookups
    for (const auto& p: m_allPeople) {
        m_employeeLocations[p->firstName] = p;
    }

    m_scheduler = std::make_unique<Scheduler>(m_config, m_plannerNode, m_employeeLocations);

    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue,
        m_config,
        m_plannerNode,
        m_employeeLocations,
        *m_scheduler
    );

    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        m_appointmentFilePaths.push_back(entry.path());
    }
    std::sort(m_appointmentFilePaths.begin(), m_appointmentFilePaths.end());
    rebuildFileQueue();

    m_ctx->addObserver(m_metricsNode);
    m_ctx->setBehaviorTree(setupBehaviorTree(m_ctx));

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Rounds: %d, Files per round: %zu",
                m_totalRounds, m_appointmentFilePaths.size());

    loadNextBatch();

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Setup Complete!");
}

bool HeadlessRunner::loadNextBatch() {
    if (m_appointmentFiles.empty()) {
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

    auto path = m_appointmentFiles.front();
    m_appointmentFiles.pop();
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Loading batch: %s (round %d/%d)",
                path.c_str(), m_currentRound + 1, m_totalRounds);

    auto appts = ConfigLoader::loadAppointmentConfig(path);
    if (!appts.has_value()) {
        RCLCPP_ERROR(rclcpp::get_logger("des_application"), "Failed to load appointments from: %s", path.c_str());
        return loadNextBatch();
    }
    m_appointments = appts.value();

    ConfigLoader::validateConfig(m_appointments, m_allPeople, m_locationMap, "5.2B_Elevator");

    m_people = ConfigLoader::filterByAppointments(m_allPeople, m_appointments);
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Simulating %zu of %zu employees",
                m_people.value().size(), m_allPeople.size());

    IAppRunner::scheduleOccupancy(*m_config, m_people.value(), m_ctx->m_rng);
    m_eventQueue.extend(IAppRunner::personArrivalGenerator(m_people.value(), "5.2B_Elevator"));
    m_eventQueue.extend(IAppRunner::createMissionQueue(m_config, m_appointments, *m_scheduler, "IMVS_Dock"));

    int firstEventTime = m_eventQueue.getFirstEventTime() - ONE_HOUR;
    int lastEventTime = m_eventQueue.getLastEventTime();

    auto latest = std::max_element(m_people.value().begin(), m_people.value().end(),
        [](const auto& a, const auto& b) { return a->departureTime < b->departureTime; });
    int lastDepartureTime = (*latest)->departureTime;

    int simEndTime = std::max(lastEventTime, lastDepartureTime) + ONE_HOUR;
    m_eventQueue.push(std::make_shared<SimulationStartEvent>(firstEventTime));
    m_eventQueue.push(std::make_shared<SimulationEndEvent>(simEndTime));

    for (auto& p : m_people.value()) {
        auto startCopy = std::make_shared<des::Person>(*p);
        startCopy->currentRoom = "OUTDOOR";
        m_eventQueue.push(std::make_shared<PersonTransitionEvent>(firstEventTime, startCopy));

        auto endCopy = std::make_shared<des::Person>(*p);
        endCopy->currentRoom = "OUTDOOR";
        m_eventQueue.push(std::make_shared<PersonTransitionEvent>(simEndTime, endCopy));
    }

    m_ctx->resetContext(m_eventQueue.getFirstEventTime());

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
