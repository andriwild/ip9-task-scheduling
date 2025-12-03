#include <QApplication>
#include <QtConcurrent/QtConcurrent>
#include <iostream>
#include <thread>

#include "init/cli_options.h"
#include "init/config_loader.h"
#include "sim/ros/path_node.h"
#include "sim/ros/marker.h"
#include "sim/scheduler.h"
#include "sim/bt_setup.h"
#include "util/types.h"
#include "view/bridge.h"
#include "view/term.h"
#include "view/gz.h"
#include "view/timeline.h"
#include "util/data.h"

constexpr int oneHour = 3600;
constexpr int SIM_START_TIME = 8 * oneHour; 
constexpr int SIM_END_TIME = SIM_START_TIME + 12 * oneHour;

void publishMarkers() {
    auto marker_node = std::make_shared<MarkerPublisher>();
    std::vector<MapLocation> rosLocations = {};
    for (auto [name, p] : locationMap) {
        rosLocations.emplace_back(name, p.m_x, p.m_y);
    }
    std::cout << "publish marker" << std::endl;
    marker_node->publishLocations(rosLocations);
}

int main(int argc, char *argv[]) {
    std::cout << "\033[1m"  <<"\n--- Descrete Event Sytem ---\n";

    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("Discrete Event System");
    QCoreApplication::setApplicationVersion("1.0");

    CliOptions opts = parseCliArguments(app);

    if (opts.stepMode) std::cout << "[Mode] Step-by-Step active.\n";
    else if (opts.delayMs > 0) std::cout << "[Mode] Animation delay: " << opts.delayMs << "ms\n";
    else std::cout << "[Mode] Fast-Forward.\n";

    auto simConfig    = ConfigLoader::loadDESConfig("../config/" + opts.simConfigPath);
    auto appointments = ConfigLoader::loadAppointmentConfig("../config/" + opts.appointmentConfigPath, SIM_START_TIME);
    if(!appointments.has_value() || !simConfig.has_value()) { 
        return 1; 
    }
    ConfigLoader::printDESConfig(simConfig.value(), opts.simConfigPath, opts.appointmentConfigPath);

    rclcpp::init(argc, argv);
    
    auto planner_node = std::make_shared<PathPlannerNode>();
    std::thread rosThread;

    if(planner_node->isReady()) {
        std::cout << "Planner ready!\n\n";
        rosThread = std::thread([planner_node] { rclcpp::spin(planner_node); });
    } else {
        std::cerr << "Planner init failed!\n";
        return 1;
    }

    EventQueue eventQueue;
    Robot robot(simConfig->robot_speed, simConfig->robot_escort_speed);
    auto tte = std::make_shared<TravelTimeEstimator>(planner_node, locationMap);
    
    SimulationContext ctx(robot, eventQueue, simConfig.value().personFindProbability, tte, employeeLocations);
    ctx.addObserver(std::make_shared<TerminalView>(TerminalView()));
    ctx.addObserver(std::make_shared<GazeboView>(GazeboView(locationMap)));

    Timeline timelineView(SIM_START_TIME, SIM_END_TIME);
    
    if(!opts.headless) {
        timelineView.show();
        auto bridge = std::make_shared<ObserverBridge>();
        QObject::connect(bridge.get(), &ObserverBridge::logReceived, &timelineView, &Timeline::handleLog);
        QObject::connect(bridge.get(), &ObserverBridge::moveReceived, &timelineView, &Timeline::handleMove);
        QObject::connect(bridge.get(), &ObserverBridge::stateChanged, &timelineView, &Timeline::handleStateChange);
        ctx.addObserver(bridge);
    }

    eventQueue.push(std::make_shared<SimulationStartEvent>(SIM_START_TIME));
    
    scheduleAppointments(appointments.value(), ctx, timelineView, employeeLocations);
    
    eventQueue.push(std::make_shared<SimulationEndEvent>(SIM_END_TIME));

    ctx.behaviorTree = setupBehaviorTree(ctx);

    publishMarkers();

    std::thread simThread([&] {
        while (!eventQueue.empty()) {
            auto e = eventQueue.top();
            eventQueue.pop();
            ctx.setTime(e->time);
            e->execute(ctx);
            
            if (opts.stepMode) {
                std::cin.get();
            } else if (opts.delayMs > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(opts.delayMs));
            }
        }
        int completeCounter = 0;
        int completeLateCounter = 0;
        int failedCounter = 0;
        for (auto& appt: appointments.value()){
            if(appt.state == des::AppointmentState::COMPLETED) completeCounter ++;
            if(appt.state == des::AppointmentState::COMPLETED_LATE) completeLateCounter++;
            if(appt.state == des::AppointmentState::FAILED) failedCounter++;
        }

        int nAppts = appointments.value().size();
        std::cout << completeCounter << " / " << nAppts << " completeCounter" << std::endl;
        std::cout << completeLateCounter << " / " << nAppts << " completeLateCounter" << std::endl;
        std::cout << failedCounter << " / " << nAppts << " failed" << std::endl;

        std::cout << "\033[1m" << "\nSimulation complete!" << std::endl;
        std::cin.get();
        QCoreApplication::quit();
    });


    QApplication::exec();
    
    if(rosThread.joinable()) rosThread.join();
    rclcpp::shutdown();
    if(simThread.joinable()) simThread.join();
    
    return 0;
}
