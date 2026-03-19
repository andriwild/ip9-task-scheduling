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
#include "../../sim/scheduler.h"
#include "event_system_msgs/srv/set_system_state.hpp"


void SimRunner::reset() {
    while (!m_eventQueue.empty()) {
        m_eventQueue.pop();
    }

    m_rosObserver->publishReset();
    m_metricsNode->clear();

    m_appointments = loadAppointments(m_config->appointmentsPath);
    m_eventQueue.extend(IAppRunner::createMissionQueue(m_config, m_appointments, *m_scheduler, "IMVS_Dock"));

    // wait until reset message is properly sendet until distribute new meetings data
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    publishMissions(m_eventQueue, m_rosObserver);
    m_ctx->resetContext(m_eventQueue.top()->time);

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "System Reset Complete");
}

void SimRunner::updateConfig(std::shared_ptr<des::SimConfig> config) {
    m_config = std::move(config);
    m_ctx->setConfig(m_config);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("des_application"), *m_config.get());
}


void SimRunner::setupApplication(const std::string& /*path*/) {
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Setup Application...");
    m_config         = m_systemConfigNode->getConfig();
    m_appointments   = loadAppointments(m_config->appointmentsPath);
    const auto people = ConfigLoader::loadEmployees(CONFIG_PATH + "employee.json");
    assert(!people.value().empty());

    for (const auto& p: people.value()) {
        m_employeeLocations[p->firstName] = p->roomLabels;
    } 
    
    m_scheduler = std::make_unique<Scheduler>(m_config, m_plannerNode, m_employeeLocations);

    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue,
        m_config,
        m_plannerNode,
        m_employeeLocations,
        *m_scheduler
    );

    IAppRunner::scheduleOccupancy(ONE_HOUR * 9, ONE_HOUR * 17, ONE_HOUR, people.value());
    m_eventQueue.extend(IAppRunner::personArrivalGenerator(people.value(),  "5.2B_Elevator"));
    m_eventQueue.extend(IAppRunner::createMissionQueue(m_config, m_appointments, *m_scheduler, "IMVS_Dock"));

    int firstEventTime = m_eventQueue.getFirstEventTime() - ONE_HOUR;
    int lastEventTime  = m_eventQueue.getLastEventTime();

    auto latest = std::max_element(people.value().begin(), people.value().end(), [](const auto& a, const auto& b) { return a->departureTime < b->departureTime; });
    int lastDepartureTime = (*latest)->departureTime;

    RCLCPP_DEBUG(rclcpp::get_logger("des_application"), "Event time range from %d to %d", firstEventTime, lastEventTime);

    m_eventQueue.push(std::make_shared<SimulationStartEvent>(firstEventTime));
    m_eventQueue.push(std::make_shared<SimulationEndEvent>(std::max(lastEventTime, lastDepartureTime) + ONE_HOUR));

    m_rosObserver = std::make_shared<RosObserver>(m_systemConfigNode);
    m_ctx->addObserver(m_metricsNode);
    m_ctx->addObserver(m_rosObserver);

    IAppRunner::publishMissions(m_eventQueue, m_rosObserver);

    m_ctx->resetContext(m_eventQueue.top()->time);

    m_ctx->m_behaviorTree = setupBehaviorTree(m_ctx);

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Setup Complete!");
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
