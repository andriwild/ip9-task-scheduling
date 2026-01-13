#pragma once

#include <QApplication>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "../behaviour/bt_setup.h"
#include "../model/context.h"
#include "../model/event.h"
#include "../observer/bridge.h"
#include "../observer/gz.h"
#include "../observer/metrics.h"
#include "../observer/term.h"
#include "../sim/ros/config.h"
#include "../sim/ros/controller.h"
#include "../sim/ros/marker.h"
#include "../sim/ros/path_node.h"
#include "../sim/scheduler.h"
#include "../util/data.h"
#include "../util/db.h"
#include "../view/timeline.h"
#include "config_loader.h"

class DesApplication
{
public:
    DesApplication(int argc, char * argv[]);
    ~DesApplication();

    int run();

private:
    bool init();
    void reset();
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
    std::shared_ptr<ObserverBridge> bridge;

    // View
    std::unique_ptr<Timeline> timelineView;
};
