#include "des_application.h"

#include <chrono>
#include <memory>
#include <optional>
#include <thread>


#include "event_system_msgs/srv/set_system_state.hpp"

std::string DEFAULT_SIM_CONFIG = "sim_config.json";
std::string DEFAULT_APPTS      = "appointments.json";

constexpr int HOUR = 3600;
constexpr int SIM_START_TIME = 8 * HOUR;
constexpr int SIM_END_TIME = SIM_START_TIME + 12 * HOUR;

DesApplication::DesApplication(int argc, char * argv[]) {
    app = std::make_unique<QApplication>(argc, argv);
    QCoreApplication::setApplicationName("Discrete Event System");
    QCoreApplication::setApplicationVersion("1.0");
}

DesApplication::~DesApplication() {
    if (rosThread.joinable()) {
        rosThread.join();
    }
    rclcpp::shutdown();
    if (simThread.joinable()) {
        simThread.join();
    }
}

std::optional<std::vector<des::Location>> DesApplication::loadPointsOfInterest() {
    auto db = DBClient("wsr_user", "wsr_password"); 
    if(db.init()) {
        std::cout << "Successful connected to DB" << std::endl;
        return db.waypoints();
    }
    return std::nullopt;
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

    for(auto poi: pointsOfInterest.value()) {
        std::cout << poi << std::endl;
        locationMap[poi.m_name] = poi.m_p;
    }

    // initialize and launch ROS nodes
    rclcpp::init(0, nullptr);
    plannerNode      = std::make_shared<PathPlannerNode>(locationMap);
    controllerNode   = std::make_shared<ControllerNode>();
    systemConfigNode = std::make_shared<ConfigNode>();

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

    timelineView = std::make_unique<Timeline>(SIM_START_TIME, SIM_END_TIME);

    return true;
}

void DesApplication::setupObservers() {
    metrics = std::make_shared<Metrics>(Metrics());
    ctx->addObserver(metrics);
    ctx->addObserver(std::make_shared<TerminalView>(TerminalView()));
    ctx->addObserver(std::make_shared<GazeboView>(GazeboView(locationMap)));

    timelineView->show();
    bridge = std::make_shared<ObserverBridge>();
    QObject::connect(bridge.get(), &ObserverBridge::logReceived, timelineView.get(), &Timeline::handleLog);
    QObject::connect(bridge.get(), &ObserverBridge::moveReceived, timelineView.get(), &Timeline::handleMove);
    QObject::connect(bridge.get(), &ObserverBridge::stateChanged, timelineView.get(), &Timeline::handleStateChange);
    ctx->addObserver(bridge);
}

void DesApplication::setupQueue(std::shared_ptr<des::SimConfig> config)
{
    robot = std::make_shared<Robot>(config->robotSpeed, config->robotEscortSpeed);
    ctx = std::make_shared<SimulationContext>(robot, eventQueue, config, plannerNode, employeeLocations);

    eventQueue.push(std::make_shared<SimulationStartEvent>(SIM_START_TIME));
    eventQueue.push(std::make_shared<SimulationEndEvent>(SIM_END_TIME));

    auto missions = scheduleAppointments(appointments.value(), employeeLocations, ctx);

    for (const auto& mission : missions) {
        double buffer = config->timeBuffer - config->missionOverhead;
        mission->time = mission->time - buffer;
        eventQueue.push(mission);
        timelineView->addMeetingPlan(mission->appointment, mission->time);
    }

    ctx->behaviorTree = setupBehaviorTree(ctx);
}

void DesApplication::reset() {
    timelineView->clear();

    while (!eventQueue.empty()) {
        eventQueue.pop();
    }

    appointments = ConfigLoader::loadAppointmentConfig("config/" + DEFAULT_APPTS, SIM_START_TIME);

    if (!appointments.has_value()) {
        std::cerr << "Failed to read config!\n\n";
        return;
    }

    auto config = std::make_shared<des::SimConfig>(systemConfigNode->currentConfig.load());
    setupQueue(config);
    setupObservers();

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

    auto config = std::make_shared<des::SimConfig>(systemConfigNode->currentConfig.load());

    setupQueue(config);
    setupObservers();

    auto applicationState = controllerNode->currentState.load();

    simThread = std::thread([&] {
        while (rclcpp::ok()) {

            if(systemConfigNode->isDirty()) {
                updateConfig(systemConfigNode->currentConfig.load());
                systemConfigNode->configUpdated();
            }
            applicationState = controllerNode->currentState.load();

            if (applicationState == event_system_msgs::srv::SetSystemState::Request::RESET) {
                reset();
                // Wait for state change to avoid multiple resets or immediate start if not desired
                while (controllerNode->currentState.load() == event_system_msgs::srv::SetSystemState::Request::RESET) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                continue;
            }

            if (eventQueue.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (applicationState == event_system_msgs::srv::SetSystemState::Request::RUN) {
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
