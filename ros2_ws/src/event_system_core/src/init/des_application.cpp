#include "des_application.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>

#include "../behaviour/bt_setup.h"
#include "../observer/gz.h"
#include "../observer/ros_observer.h"
#include "../observer/term.h"
#include "../sim/scheduler.h"
#include "../util/db.h"
#include "config_loader.h"
#include "event_system_msgs/srv/set_system_state.hpp"

std::string DEFAULT_SIM_CONFIG = "sim_config.json";
std::string DEFAULT_APPTS = "appointments.json";

constexpr int HOUR = 3600;
constexpr int SIM_START_TIME = 8 * HOUR;
constexpr int SIM_END_TIME = SIM_START_TIME + 12 * HOUR;

std::optional<std::vector<des::Location>> DesApplication::loadPointsOfInterest() {
    auto db = DBClient("wsr_user", "wsr_password");
    if (db.init()) {
        std::cout << "Successful connected to DB" << std::endl;
        return db.waypoints();
    }
    return std::nullopt;
}

bool DesApplication::initROS() {
    rclcpp::init(0, nullptr);
    plannerNode      = std::make_shared<PathPlannerNode>(locationMap);
    controllerNode   = std::make_shared<ControllerNode>();
    systemConfigNode = std::make_shared<ConfigNode>(robot, ctx);

    if (!plannerNode->isReady()) {
        std::cerr << "Planner init failed!\n";
        return false;
    }
    std::cout << "Planner ready!\n\n";

    rosThread = std::thread([this] {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(plannerNode);
        executor.add_node(controllerNode);
        executor.add_node(systemConfigNode);
        executor.spin();
    });
    return true;
}

bool DesApplication::init() {
    appointments = ConfigLoader::loadAppointmentConfig("config/" + DEFAULT_SIM_CONFIG, SIM_START_TIME);
    pointsOfInterest = loadPointsOfInterest();

    if (!appointments.has_value()) {
        std::cerr << "Failed to read config!\n\n";
        return false;
    }

    if (!pointsOfInterest.has_value()) {
        std::cerr << "Failed to connect DB!\n\n";
        return false;
    }

    if (!initROS()) {
        std::cerr << "Failed to create ROS Nodes!\n\n";
        return false;
    }

    return true;
}

void DesApplication::setupObservers() {
    metrics = std::make_shared<Metrics>(Metrics());
    rosObserver = std::make_shared<RosObserver>(systemConfigNode);

    ctx->addObserver(metrics);
    ctx->addObserver(rosObserver);
    ctx->addObserver(std::make_shared<TerminalView>(TerminalView()));
    ctx->addObserver(std::make_shared<GazeboView>(GazeboView(locationMap)));
}

void DesApplication::setupQueue(std::shared_ptr<des::SimConfig> config) {
    eventQueue.push(std::make_shared<SimulationStartEvent>(SIM_START_TIME));
    eventQueue.push(std::make_shared<SimulationEndEvent>(SIM_END_TIME));

    auto missions = scheduleAppointments(appointments.value(), employeeLocations, ctx);

    for (const auto& mission : missions) {
        double buffer = config->timeBuffer - config->missionOverhead;
        mission->time = mission->time - buffer;
        eventQueue.push(mission);
        if (rosObserver) {
            rosObserver->publishMeeting(mission->appointment, mission->time);
        }
    }

    ctx->behaviorTree = setupBehaviorTree(ctx);
}

void DesApplication::reset(std::shared_ptr<des::SimConfig> config) {
    std::cout << "Reset ..." << std::endl;
    while (!eventQueue.empty()) {
        eventQueue.pop();
    }

    if (rosObserver) {
        std::cout << "Clean up Timeline" << std::endl;
        rosObserver->publishReset();
    }

    appointments = ConfigLoader::loadAppointmentConfig("config/" + config->appointmentsPath, SIM_START_TIME);

    if (!appointments.has_value()) {
        std::cerr << "Failed to read config!\n\n";
        return;
    }

    std::cout << "Setup Queue" << std::endl;
    setupQueue(config);

    ctx->resetTime(SIM_START_TIME);

    std::cout << "System Reset Complete. Waiting for Start..." << std::endl;
}

void DesApplication::updateConfig(des::SimConfig config) {
    robot->setDefaultSpeed(config.robotSpeed);
    robot->setAccompanytSpeed(config.robotEscortSpeed);
    ctx->setConfig(config);
    std::cout << config << std::endl;
}

int DesApplication::run() {
    std::cout << "\033[1m" << "\n----- Descrete Event Sytem -----\n" << "\033[0m";

    if (!init()) {
        return 1;
    }

    robot  = std::make_shared<Robot>(config->robotSpeed, config->robotEscortSpeed);
    config = std::make_shared<des::SimConfig>(systemConfigNode->getConfig());

    ctx = std::make_shared<SimulationContext>(
        robot,
        eventQueue,
        config,
        plannerNode,
        employeeLocations
    );

    setupQueue(config);
    setupObservers();

    auto applicationState = controllerNode->currentState.load();

    simThread = std::thread([&] {

        while (rclcpp::ok()) {
            applicationState = controllerNode->currentState.load();

            if (applicationState == event_system_msgs::srv::SetSystemState::Request::RESET) {
                reset(config);
                controllerNode->currentState.store(event_system_msgs::srv::SetSystemState::Request::PAUSE);
            }

            if (applicationState == event_system_msgs::srv::SetSystemState::Request::RUN && !eventQueue.empty()) {
                auto e = eventQueue.top();
                eventQueue.pop();
                ctx->setTime(e->time);
                e->execute(*ctx);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        std::cout << "\033[1m" << "\nSimulation complete!" << "\033[0m" << std::endl;

        metrics->show();

        std::cin.get();
        QCoreApplication::quit();
        QApplication::quit();
    });

    return app->exec();
}
