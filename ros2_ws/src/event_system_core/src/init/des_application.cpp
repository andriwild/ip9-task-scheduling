#include "des_application.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
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

bool DesApplication::loadPointsOfInterest(bool printPOIS = false) {
    auto db = DBClient("wsr_user", "wsr_password");
    if (db.init()) {
        std::cout << "Successful connected to DB" << std::endl;
    }

    pointsOfInterest =  db.waypoints();
    if (!pointsOfInterest.has_value()) {
        std::cout << "No points of interest loaded!" << std::endl;
        return false;
    }
    

    for (auto poi : pointsOfInterest.value()) {
        locationMap[poi.m_name] = poi.m_p;
        if(printPOIS) {
            std::cout << poi << std::endl;
        }
    }
    std::cout << "Successful loaded points of interest!" << std::endl;
    return true;
}

bool DesApplication::initROS() {
    rclcpp::init(0, nullptr);
    plannerNode      = std::make_shared<PathPlannerNode>(locationMap);
    controllerNode   = std::make_shared<ControllerNode>();
    systemConfigNode = std::make_shared<ConfigNode>();

    if (!plannerNode->isReady()) {
        std::cerr << "Planner init failed!\n";
        return false;
    }
    std::cout << "Planner ready!\n";

    rosThread = std::thread([this] {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(plannerNode);
        executor.add_node(controllerNode);
        executor.add_node(systemConfigNode);
        executor.spin();
    });
    std::cout << "Launched all ROS Nodes!" << std::endl;
    return true;
}

bool DesApplication::loadAppointments() {
    std::cout << "Load Appointments: " << config->appointmentsPath << std::endl;
    appointments = ConfigLoader::loadAppointmentConfig("config/" + config->appointmentsPath, SIM_START_TIME);
    if (!appointments.has_value()) {
        return false;
    }

    std::cout << "Successful loaded appointments! (" << appointments.value().size()<< ")" << std::endl;
    return true;
}

void DesApplication::setupObservers() {
    metrics     = std::make_shared<Metrics>(Metrics());
    rosObserver = std::make_shared<RosObserver>(systemConfigNode);

    ctx->addObserver(metrics);
    ctx->addObserver(rosObserver);
    ctx->addObserver(std::make_shared<TerminalView>(TerminalView()));
    ctx->addObserver(std::make_shared<GazeboView>(GazeboView(locationMap)));
}

void DesApplication::setupQueue(std::shared_ptr<des::SimConfig> config) {
    std::cout << "Start filling event queue...";
    eventQueue.push(std::make_shared<SimulationStartEvent>(SIM_START_TIME));
    eventQueue.push(std::make_shared<SimulationEndEvent>(SIM_END_TIME));

    auto missions = scheduleAppointments(appointments.value(), employeeLocations, ctx);

    int pubCounter = 0;
    for (const auto& mission : missions) {
        double buffer = config->timeBuffer - config->missionOverhead;
        mission->time = mission->time - buffer;
        eventQueue.push(mission);
        if (rosObserver) {
            rosObserver->publishMeeting(mission->appointment, mission->time);
            pubCounter++;
        }
    }

    std::cout << " - Done! ("<< pubCounter << ")" << std::endl;
}

void DesApplication::reset(std::shared_ptr<des::SimConfig> config) {
    while (!eventQueue.empty()) {
        eventQueue.pop();
    }

    if (rosObserver) {
        rosObserver->publishReset();
    }

    appointments = ConfigLoader::loadAppointmentConfig("config/" + config->appointmentsPath, SIM_START_TIME);

    if (!appointments.has_value()) {
        std::cerr << "Failed to read config!\n\n";
        return;
    }

    setupQueue(config);
    ctx->resetTime(SIM_START_TIME);
    std::cout << "System Reset Complete. Waiting for Start..." << std::endl;
}

void DesApplication::updateConfig(std::shared_ptr<des::SimConfig> newConfig) {
    config = newConfig;
    robot->setDefaultSpeed(config->robotSpeed);
    robot->setAccompanytSpeed(config->robotAccompanySpeed);
    ctx->setConfig(config);
    std::cout << *config.get() << std::endl;
}

int DesApplication::run() {
    std::cout << "\033[1m" << "\n----- Descrete Event Sytem -----\n" << "\033[0m";

    if (!loadPointsOfInterest()) {
        std::cerr << "Failed to load points of interest!\n";
        return 1;
    }

    if (!initROS()) {
        std::cerr << "Failed to create ROS Nodes!\n";
        return 1;
    }

    config = systemConfigNode->getConfig();
    robot  = std::make_shared<Robot>(config->robotSpeed, config->robotAccompanySpeed);

    if (!loadAppointments()) {
        std::cerr << "Failed to load appointments!\n";
        return 1;
    }

    ctx = std::make_shared<SimulationContext>(
        robot,
        eventQueue,
        config,
        plannerNode,
        employeeLocations
    );

    systemConfigNode->setRobot(robot);
    systemConfigNode->setContext(ctx);

    setupObservers();
    setupQueue(config);
    ctx->behaviorTree = setupBehaviorTree(ctx);

    std::cout << "Start Simulation Thread" << std::endl;
    simThread = std::thread([&] {
        while (rclcpp::ok()) {


            if(systemConfigNode->isConfigDirty()){
                updateConfig(systemConfigNode->getConfig());
                systemConfigNode->clearDirty();
            }

            switch(controllerNode->currentState.load()) {
                case SystemState::Request::RESET:
                    reset(config);
                    controllerNode->currentState.store(event_system_msgs::srv::SetSystemState::Request::PAUSE);
                    break;

                case SystemState::Request::PAUSE:
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    break;

                case SystemState::Request::RUN:
                    if(!eventQueue.empty()) {
                        auto e = eventQueue.top();
                        eventQueue.pop();
                        ctx->setTime(e->time);
                        e->execute(*ctx);
                    }
                    break;
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
