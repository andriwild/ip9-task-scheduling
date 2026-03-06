#include "headless_runner.h"
#include "../../behaviour/bt_setup.h"
#include "event_system_msgs/srv/detail/set_system_state__struct.hpp"

void HeadlessRunner::setupQueue(const std::shared_ptr<des::SimConfig> &config) {
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
    }
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Event queue: (%zu) events inserted (inkl. Start and End)", m_eventQueue.size());
}

void HeadlessRunner::initROS() {
    m_plannerNode = std::make_shared<PathPlannerNode>(m_locationMap);
    m_metricsNode = std::make_shared<MetricsNode>();

    // leads to spam messages on lower logger level
    rclcpp::get_logger("event_system_planner_node.rclcpp_action").set_level(rclcpp::Logger::Level::Warn);

    if (!m_plannerNode->isReady()) {
        throw std::runtime_error("Nav2 Planner initialization failed");
    }
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Planner ready");

    m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    m_rosThread = std::thread([this] {
        m_executor->add_node(m_plannerNode);
        m_executor->add_node(m_metricsNode);
        m_executor->spin();
        m_executor->remove_node(m_plannerNode);
        m_executor->remove_node(m_metricsNode);
    });
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Launched all ROS Nodes");
}

void HeadlessRunner::setupObservers(bool headless, bool verbose) {
    m_ctx->addObserver(m_metricsNode);
}

void HeadlessRunner::setupApplication() {
    try {
        loadPointsOfInterest();
        initROS();
        auto config = ConfigLoader::loadSimConfig(CONFIG_PATH + "sim_config.json").value();
        m_config = std::make_shared<des::SimConfig>(config);
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

int HeadlessRunner::loadAppState() const {
    return event_system_msgs::srv::SetSystemState::Request::RUN;
}

void HeadlessRunner::reset() {
    while (!m_eventQueue.empty()) {
        m_eventQueue.pop();
    }
    // wait until reset message is properly sendet until distribute new meetings data
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    loadAppointments(m_config->appointmentsPath);
    setupQueue(m_config);
    m_ctx->resetContext(m_eventQueue.top()->time);

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "System Reset Complete");
}

void HeadlessRunner::enterPause() const {
    // not implemented yet
}
void HeadlessRunner::updateConfig() {
    // not implemented yet
}