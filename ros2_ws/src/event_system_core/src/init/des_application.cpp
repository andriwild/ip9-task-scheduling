#include "des_application.h"
#include <chrono>
#include <memory>
#include <thread>

#include "event_system_msgs/srv/set_system_state.hpp"

constexpr int HOUR = 3600;
constexpr int SIM_START_TIME = 8 * HOUR;
constexpr int SIM_END_TIME = SIM_START_TIME + 12 * HOUR;

DesApplication::DesApplication(int argc, char *argv[]) {
    app = std::make_unique<QApplication>(argc, argv);
    QCoreApplication::setApplicationName("Discrete Event System");
    QCoreApplication::setApplicationVersion("1.0");
    opts = parseCliArguments(*app);
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

bool DesApplication::init() {
    appointments = ConfigLoader::loadAppointmentConfig(
        "config/" + opts.appointmentConfigPath, 
        SIM_START_TIME
    );

    if (!appointments.has_value()) {
        std::cerr << "Failed to read config!\n\n";
        return false;
    }

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



    if (!opts.headless) {
        timelineView->show();
        bridge = std::make_shared<ObserverBridge>();
        QObject::connect(bridge.get(), &ObserverBridge::logReceived,  timelineView.get(), &Timeline::handleLog);
        QObject::connect(bridge.get(), &ObserverBridge::moveReceived, timelineView.get(), &Timeline::handleMove);
        QObject::connect(bridge.get(), &ObserverBridge::stateChanged, timelineView.get(), &Timeline::handleStateChange);
        ctx->addObserver(bridge);
    }
}


void DesApplication::setupQueue(std::shared_ptr<des::SimConfig> config) {
    robot = std::make_shared<Robot>(config->robotSpeed, config->robotEscortSpeed);
    ctx   = std::make_shared<SimulationContext>(robot, eventQueue, config, plannerNode, employeeLocations);

    eventQueue.push(std::make_shared<SimulationStartEvent>(SIM_START_TIME));
    eventQueue.push(std::make_shared<SimulationEndEvent>(SIM_END_TIME));

    auto missions = scheduleAppointments(appointments.value(), employeeLocations, ctx);

    for (const auto &mission : missions) {
        double buffer = config->timeBuffer - config->missionOverhead;
        mission->time = mission->time - buffer;
        eventQueue.push(mission);
        timelineView->addMeetingPlan(mission->appointment, mission->time);
    }

    ctx->behaviorTree = setupBehaviorTree(ctx);

}

int DesApplication::run() {
    std::cout << "\033[1m" << "\n----- Descrete Event Sytem -----\n";

    if (!init()) {
        return 1;
    }

    auto config = std::make_shared<des::SimConfig>(systemConfigNode->currentConfig.load());

    setupQueue(config);
    setupObservers();

    auto applicationState = controllerNode->currentState.load();

    simThread = std::thread([&] {
        while (rclcpp::ok() && !eventQueue.empty()) {
            if (applicationState == event_system_msgs::srv::SetSystemState::Request::RUN) {
                auto e = eventQueue.top();
                eventQueue.pop();
                ctx->setTime(e->time);
                e->execute(*ctx);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            applicationState = controllerNode->currentState.load();
        }

        std::cout << "\033[1m" << "\nSimulation complete!" << std::endl;

        metrics->show();

        std::cin.get();
        QCoreApplication::quit();
        QApplication::quit();
    });

    return app->exec();
}
