#include "des_application.h"
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

#include "../behaviour/bt_setup.h"
#include "../observer/gz.h"
#include "../observer/ros.h"
#include "../observer/term.h"
#include "../sim/scheduler.h"
#include "../util/db.h"
#include "config_loader.h"
#include "event_system_msgs/srv/set_system_state.hpp"

constexpr int ONE_HOUR = 3600;
const std::string CONFIG_PATH = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/";


void DesApplication::loadPointsOfInterest() {
    auto db = DBClient("wsr_user", "wsr_password");
    if (!db.init()) {
        throw std::runtime_error("Could not connect DB");
    }
    RCLCPP_INFO(m_node->get_logger(), "Successful connected to DB");

    const auto pois = db.waypoints();
    if (!pois.has_value()) {
        throw std::runtime_error("Could not load points of interest from file");
    }

    for (auto poi : pois.value()) {
        m_locationMap[poi.m_name] = poi.m_p;
        RCLCPP_DEBUG_STREAM(m_node->get_logger(), poi);
    }
    RCLCPP_INFO(m_node->get_logger(), "Successful loaded %zu points of interest", m_locationMap.size());
}

void DesApplication::initROS() {
    m_plannerNode      = std::make_shared<PathPlannerNode>(m_locationMap);
    m_controllerNode   = std::make_shared<ControllerNode>();
    m_systemConfigNode = std::make_shared<ConfigNode>();
    m_metricsNode      = std::make_shared<MetricsNode>();

    m_plannerNode->get_logger().set_level(LOG_LEVEL);
    rclcpp::get_logger("event_system_planner_node.rclcpp_action").set_level(rclcpp::Logger::Level::Warn);
    m_controllerNode->get_logger().set_level(LOG_LEVEL);
    m_systemConfigNode->get_logger().set_level(LOG_LEVEL);
    m_metricsNode->get_logger().set_level(LOG_LEVEL);

    if (!m_plannerNode->isReady()) {
        throw std::runtime_error("Nav2 Planner initialization failed");
    }
    RCLCPP_INFO(m_node->get_logger(), "Planner ready");

    m_rosThread = std::thread([this] {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(m_plannerNode);
        executor.add_node(m_controllerNode);
        executor.add_node(m_systemConfigNode);
        executor.add_node(m_metricsNode);
        executor.spin();
    });
    RCLCPP_INFO(m_node->get_logger(), "Launched all ROS Nodes");
}

void DesApplication::loadAppointments(const std::string& path) {
    RCLCPP_INFO(m_node->get_logger(), "Load Appointments: %s", m_config->appointmentsPath.c_str());
    const auto appts = ConfigLoader::loadAppointmentConfig(CONFIG_PATH + path);
    if (!appts.has_value()) {
        throw std::runtime_error("Could not load appointments from file");
    }
    m_appointments = appts.value();
    RCLCPP_INFO(m_node->get_logger(), "Successful loaded %zu appointments", m_appointments.size());
}

void DesApplication::loadEmployeeLocations() {
    RCLCPP_DEBUG(m_node->get_logger(), "Load Employee Locations");
    const auto locations = ConfigLoader::loadEmployeeLocations(CONFIG_PATH + "employee_locations.json");
    if (!locations.has_value()) {
        throw std::runtime_error("Could not load appointments from file");
    }
    m_employeeLocations = locations.value();
    RCLCPP_INFO(m_node->get_logger(), "Successful loaded employee locations (%zu)", m_employeeLocations.size());
}

void DesApplication::setupObservers(const bool headless = true, const bool verbose = false) {
    m_rosObserver = std::make_shared<RosObserver>(m_systemConfigNode);

    m_ctx->addObserver(m_metricsNode);
    m_ctx->addObserver(m_rosObserver);
    if (verbose) { m_ctx->addObserver(std::make_shared<TerminalView>(TerminalView())); }
    if (!headless) { m_ctx->addObserver(std::make_shared<GazeboView>(GazeboView(m_locationMap))); }
}

void DesApplication::setupQueue(const std::shared_ptr<des::SimConfig> &config) {
    RCLCPP_DEBUG(m_node->get_logger(), "Start filling event queue");

    const auto missions = m_scheduler->simplePlan(m_appointments, m_ctx->m_robot->getIdleLocation());

    int firstEventTime = missions.front()->time - ONE_HOUR;
    int lastEventTime = missions.back()->time + ONE_HOUR;

    RCLCPP_DEBUG(m_node->get_logger(), "Event time range from %d to %d", firstEventTime, lastEventTime);

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
    RCLCPP_INFO(m_node->get_logger(), "Event queue: (%zu) events inserted", m_eventQueue.size());
}

void DesApplication::reset() {
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

    RCLCPP_INFO(m_node->get_logger(), "System Reset Complete");
}

void DesApplication::updateConfig(std::shared_ptr<des::SimConfig> config) {
    m_config = std::move(config);
    m_ctx->setConfig(m_config);
    RCLCPP_DEBUG_STREAM(m_node->get_logger(), *m_config.get());
}

void DesApplication::setupApplication() {
    try {
        loadPointsOfInterest();
        initROS();
        m_config = m_systemConfigNode->getConfig();
        loadAppointments(m_config->appointmentsPath);
        loadEmployeeLocations();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(m_node->get_logger(), "Exception: %s", e.what());
        exit(EXIT_FAILURE);
    }

    m_scheduler = std::make_unique<Scheduler>(m_config, m_plannerNode, m_employeeLocations, m_node->get_logger());

    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue,
        m_config,
        m_plannerNode,
        m_employeeLocations,
        *m_scheduler,
        m_node->get_logger()
    );

    setupObservers();
    setupQueue(m_config);
    m_ctx->resetContext(m_eventQueue.top()->time);

    RCLCPP_DEBUG(m_node->get_logger(), "Create Behaviour Tree");
    m_ctx->m_behaviorTree = setupBehaviorTree(m_ctx);
    RCLCPP_INFO(m_node->get_logger(), "Behaviour Tree created");
}

void DesApplication::updateConfig() {
    if (m_systemConfigNode->isConfigDirty()) {
        updateConfig(m_systemConfigNode->getConfig());
        m_systemConfigNode->clearDirty();
    }
}

void DesApplication::enterPause() const {
    m_controllerNode->currentState.store(SystemState::Request::PAUSE);
    RCLCPP_DEBUG(m_node->get_logger(), "Simulation loop paused");
}

int DesApplication::loadAppState() const {
    return m_controllerNode->currentState.load();
}
