#include "sim_runner.h"
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <thread>
#include <utility>

#include "../../behaviour/bt_setup.h"
#include "../../observer/gz.h"
#include "../../observer/ros.h"
#include "../../observer/term.h"
#include "../../sim/scheduler.h"
#include "../../util/db.h"
#include "event_system_msgs/srv/set_system_state.hpp"

void SimRunner::initROS() {
    m_plannerNode      = std::make_shared<PathPlannerNode>(m_locationMap);
    m_controllerNode   = std::make_shared<ControllerNode>();
    m_systemConfigNode = std::make_shared<ConfigNode>();
    m_metricsNode      = std::make_shared<MetricsNode>();

    // leads to spam messages on lower logger level
    rclcpp::get_logger("event_system_planner_node.rclcpp_action").set_level(rclcpp::Logger::Level::Warn);

    if (!m_plannerNode->isReady()) {
        throw std::runtime_error("Nav2 Planner initialization failed");
    }
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Planner ready");

    m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    m_rosThread = std::thread([this] {
        m_executor->add_node(m_plannerNode);
        m_executor->add_node(m_controllerNode);
        m_executor->add_node(m_systemConfigNode);
        m_executor->add_node(m_metricsNode);
        m_executor->spin();
        m_executor->remove_node(m_plannerNode);
        m_executor->remove_node(m_controllerNode);
        m_executor->remove_node(m_systemConfigNode);
        m_executor->remove_node(m_metricsNode);
    });
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Launched all ROS Nodes");
}


void SimRunner::setupObservers(const bool headless, const bool verbose = false) {
    m_rosObserver = std::make_shared<RosObserver>(m_systemConfigNode);
    m_ctx->addObserver(m_metricsNode);
    m_ctx->addObserver(m_rosObserver);
    if (verbose) { m_ctx->addObserver(std::make_shared<TerminalView>(TerminalView())); }
    if (!headless) { m_ctx->addObserver(std::make_shared<GazeboView>(GazeboView(m_locationMap))); }
}

void SimRunner::setupQueue(const std::shared_ptr<des::SimConfig> &config) {
    RCLCPP_DEBUG(rclcpp::get_logger("des_application"), "Start filling event queue");

    const auto missions = m_scheduler->simplePlan(m_appointments, m_ctx->m_robot->getIdleLocation());

    int firstEventTime = missions.front()->time - ONE_HOUR;
    int lastEventTime = missions.back()->time + ONE_HOUR;

    RCLCPP_DEBUG(rclcpp::get_logger("des_application"), "Event time range from %d to %d", firstEventTime, lastEventTime);

    m_eventQueue.push(std::make_shared<SimulationStartEvent>(firstEventTime));
    m_eventQueue.push(std::make_shared<SimulationEndEvent>(lastEventTime));

    for (const auto& mission : missions) {
        const double buffer = config->timeBuffer;
        mission->time = mission->time - buffer;
        m_eventQueue.push(mission);
        if (m_rosObserver) {
            m_rosObserver->publishMeeting(mission->appointment, mission->time);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Event queue: (%zu) events inserted (inkl. Start and End)", m_eventQueue.size());
}

void SimRunner::reset() {
    while (!m_eventQueue.empty()) {
        m_eventQueue.pop();
    }

    m_rosObserver->publishReset();
    m_metricsNode->clear();

    // wait until reset message is properly sendet until distribute new meetings data
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    loadAppointments(m_config->appointmentsPath);
    setupQueue(m_config);
    m_ctx->resetContext(m_eventQueue.top()->time);

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "System Reset Complete");
}

void SimRunner::updateConfig(std::shared_ptr<des::SimConfig> config) {
    m_config = std::move(config);
    m_ctx->setConfig(m_config);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("des_application"), *m_config.get());
}

void SimRunner::setupApplication() {
    try {
        loadPointsOfInterest();
        initROS();
        m_config = m_systemConfigNode->getConfig();
        loadAppointments(m_config->appointmentsPath);
        loadEmployeeLocations();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("des_application"), "Exception: %s", e.what());
        exit(EXIT_FAILURE);
    }

    m_scheduler = std::make_unique<Scheduler>(m_config, m_plannerNode, m_employeeLocations);

    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue,
        m_config,
        m_plannerNode,
        m_employeeLocations,
        *m_scheduler
    );

    setupObservers(true, false); // (headless: no gz, verbose: no term obs)
    setupQueue(m_config);
    m_ctx->resetContext(m_eventQueue.top()->time);

    RCLCPP_DEBUG(rclcpp::get_logger("des_application"), "Create Behaviour Tree");
    m_ctx->m_behaviorTree = setupBehaviorTree(m_ctx);
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Behaviour Tree created");
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
    auto state = m_controllerNode->currentState.load();
    return state;
}