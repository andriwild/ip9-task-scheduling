#pragma once

#include <QApplication>
#include <memory>
#include <thread>
#include <vector>

#include "../model/context.h"
#include "../model/event.h"
#include "../observer/metrics.h"
#include "../observer/ros.h"
#include "../sim/ros/config.h"
#include "../sim/ros/controller.h"
#include "../sim/ros/marker.h"
#include "../sim/ros/path_node.h"
#include "../util/data.h"

class DesApplication {
public:

    DesApplication(int argc, char * argv[]) {
        app = std::make_unique<QApplication>(argc, argv);
        QCoreApplication::setApplicationName("Discrete Event System");
        QCoreApplication::setApplicationVersion("1.0");
    }

    ~DesApplication() {
        if (rosThread.joinable()) {
            rosThread.join();
        }
        rclcpp::shutdown();
        if (simThread.joinable()) {
            simThread.join();
        }
    }

    int run();

private:
    bool init();
    bool initROS();
    void reset(std::shared_ptr<des::SimConfig> config);
    void setupSimulation();
    void setupObservers();
    void updateConfig(des::SimConfig config);
    void setupQueue(std::shared_ptr<des::SimConfig> config);
    std::optional<std::vector<des::Location>> loadPointsOfInterest();

    // Qt Application
    std::unique_ptr<QApplication> app;

    // Configuration
    std::optional<std::vector<std::shared_ptr<des::Appointment>>> appointments;

    // Simulation Data
    std::shared_ptr<SimulationContext> ctx;
    std::shared_ptr<des::SimConfig> config;
    EventQueue eventQueue;
    std::shared_ptr<Robot> robot;
    std::optional<std::vector<des::Location>> pointsOfInterest;

    // ROS
    std::shared_ptr<PathPlannerNode> plannerNode;
    std::shared_ptr<ControllerNode> controllerNode;
    std::shared_ptr<ConfigNode> systemConfigNode;
    std::thread rosThread;
    std::thread simThread;

    // Observers
    std::shared_ptr<Metrics> metrics;

    // View
    std::shared_ptr<RosObserver> rosObserver;
};
