#include "des_application.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>

#include "../behaviour/bt_setup.h"
#include "../observer/gz.h"
#include "../observer/ros.h"
#include "../observer/term.h"
#include "../sim/scheduler.h"
#include "../util/db.h"
#include "config_loader.h"
#include "event_system_msgs/srv/set_system_state.hpp"

constexpr int HOUR = 3600;
constexpr int SIM_START_TIME = 8 * HOUR;
constexpr int SIM_END_TIME = SIM_START_TIME + 12 * HOUR;
const std::string CONFIG_PATH = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/";

void DesApplication::loadPointsOfInterest(bool printPOIS = false) {
    auto db = DBClient("wsr_user", "wsr_password");
    if (!db.init()) {
        throw std::runtime_error("Could not connect DB!");
    }
    std::cout << "Successful connected to DB" << std::endl;

    auto pois = db.waypoints();
    if (!pois.has_value()) {
        throw std::runtime_error("Could not laod points of interest from file!");
    }

    for (auto poi : pois.value()) {
        m_locationMap[poi.m_name] = poi.m_p;
        if (printPOIS) { std::cout << poi << std::endl; }
    }
    std::cout << "Successful loaded points of interest!" << std::endl;
}

void DesApplication::initROS() {
    rclcpp::init(0, nullptr);
    m_plannerNode      = std::make_shared<PathPlannerNode>(m_locationMap);
    m_controllerNode   = std::make_shared<ControllerNode>();
    m_systemConfigNode = std::make_shared<ConfigNode>();
    m_metricsNode      = std::make_shared<MetricsNode>();

    if (!m_plannerNode->isReady()) {
        throw std::runtime_error("Nav2 Planner initialization failed");
    }
    std::cout << "Planner ready!\n";

    m_rosThread = std::thread([this] {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(m_plannerNode);
        executor.add_node(m_controllerNode);
        executor.add_node(m_systemConfigNode);
        executor.add_node(m_metricsNode);
        executor.spin();
    });
    std::cout << "Launched all ROS Nodes!" << std::endl;
}

void DesApplication::loadAppointments(std::string path) {
    std::cout << "Load Appointments: " << m_config->appointmentsPath << std::endl;
    auto appts = ConfigLoader::loadAppointmentConfig(CONFIG_PATH + path, SIM_START_TIME);
    if (!appts.has_value()) {
        throw std::runtime_error("Could not laod appointments from file!");
    }
    m_appointments = appts.value();
    std::cout << "Successful loaded " << m_appointments.size() << " appointments!" << std::endl;
}

void DesApplication::loadEmployeeLocations() {
    std::cout << "Load Employee Locations..." << std::endl;
    auto locations = ConfigLoader::loadEmployeeLocations(CONFIG_PATH + "employee_locations.json");
    if (!locations.has_value()) {
        throw std::runtime_error("Could not laod appointments from file!");
    }
    m_employeeLocations = locations.value();
    std::cout << "Successful loaded employee locations! (" << m_employeeLocations.size() << ")" << std::endl;
}

void DesApplication::setupObservers() {
    m_rosObserver = std::make_shared<RosObserver>(m_systemConfigNode);

    m_ctx->addObserver(m_metricsNode);
    m_ctx->addObserver(m_rosObserver);
    m_ctx->addObserver(std::make_shared<TerminalView>(TerminalView()));
    // ctx->addObserver(std::make_shared<GazeboView>(GazeboView(m_locationMap)));
}

void DesApplication::setupQueue(std::shared_ptr<des::SimConfig> config) {
    std::cout << "Start filling event queue...";
    m_eventQueue.push(std::make_shared<SimulationStartEvent>(SIM_START_TIME));
    m_eventQueue.push(std::make_shared<SimulationEndEvent>(SIM_END_TIME));

    auto missions = scheduleAppointments(m_appointments, m_employeeLocations, m_ctx);

    for (const auto& mission : missions) {
        double buffer = config->timeBuffer + config->missionOverhead;
        mission->time = mission->time - buffer;
        m_eventQueue.push(mission);
        if (m_rosObserver) {
            m_rosObserver->publishMeeting(mission->appointment, mission->time);
        }
    }
    std::cout << " - Done!" << std::endl;
}

void DesApplication::reset() {
    while (!m_eventQueue.empty()) {
        m_eventQueue.pop();
    }

    m_rosObserver->publishReset();
    m_metricsNode->clear();
    
    loadAppointments(m_config->appointmentsPath);

    setupQueue(m_config);
    m_ctx->resetTime(SIM_START_TIME);
    std::cout << "System Reset Complete" << std::endl;
}

void DesApplication::updateConfig(std::shared_ptr<des::SimConfig> newConfig) {
    m_config = newConfig;
    m_ctx->setConfig(m_config);
    std::cout << *m_config.get() << std::endl;
}

int DesApplication::run() {
    std::cout << "\033[1m" << "\n----- Descrete Event Sytem -----\n" << "\033[0m";

    try {
        loadPointsOfInterest(true);
        initROS();
        m_config = m_systemConfigNode->getConfig();
        loadAppointments(m_config->appointmentsPath);
        loadEmployeeLocations();
    } catch (const std::exception& e) {
        exit(EXIT_FAILURE);
    }

    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue,
        m_config,
        m_plannerNode,
        m_employeeLocations
    );

    setupObservers();
    setupQueue(m_config);
    m_ctx->behaviorTree = setupBehaviorTree(m_ctx);

    m_simThread = std::thread([&] {
        std::cout << "Start Simulation Thread" << std::endl;
        while (rclcpp::ok()) {

            if (m_systemConfigNode->isConfigDirty()) {
                updateConfig(m_systemConfigNode->getConfig());
                m_systemConfigNode->clearDirty();
            }

            switch (m_controllerNode->currentState.load()) {
                case SystemState::Request::RESET:
                    reset();
                    m_controllerNode->currentState.store(event_system_msgs::srv::SetSystemState::Request::PAUSE);
                    break;

                case SystemState::Request::PAUSE:
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    break;

                case SystemState::Request::RUN:
                    if (!m_eventQueue.empty()) {
                        auto e = m_eventQueue.top();
                        m_eventQueue.pop();
                        m_ctx->setTime(e->time);
                        e->execute(*m_ctx);
                    }
                    break;
            }
        }

        std::cout << "\033[1m" << "\nSimulation complete!" << "\033[0m" << std::endl;

        QCoreApplication::quit();
        QApplication::quit();
    });

    return m_app->exec();
}
