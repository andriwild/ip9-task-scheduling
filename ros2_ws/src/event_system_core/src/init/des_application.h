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
#include "../sim/ros/path_node.h"
#include "../sim/scheduler.h"

class DesApplication {
public:
    DesApplication(int argc, char * argv[]) {
        m_app = std::make_unique<QApplication>(argc, argv);
        QCoreApplication::setApplicationName("Discrete Event System");
        QCoreApplication::setApplicationVersion("1.0");

        rclcpp::init(argc, argv);
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "\n----- Descrete Event Sytem -----");
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "C++ Version: %ld", __cplusplus);
    }

    ~DesApplication() {
        if (m_rosThread.joinable()) {
            m_rosThread.join();
        }
        rclcpp::shutdown();
        if (m_simThread.joinable()) {
            m_simThread.join();
        }
    }

    void loadAppointments(const std::string& path);
    void loadPointsOfInterest();
    void loadEmployeeLocations();
    void initROS();
    void reset();
    void setupObservers(bool headless, bool verbose);
    void updateConfig(std::shared_ptr<des::SimConfig> config);
    void setupQueue(const std::shared_ptr<des::SimConfig> &config);
    void setupApplication();
    int loadAppState() const;
    void updateConfig();
    void enterPause() const;


    EventQueue m_eventQueue;
    std::shared_ptr<SimulationContext> m_ctx;
    std::unique_ptr<QApplication> m_app;

private:
    std::unique_ptr<Scheduler> m_scheduler;
    std::vector<std::shared_ptr<des::Appointment>> m_appointments;
    std::shared_ptr<des::SimConfig> m_config;
    std::map<std::string, des::Point> m_locationMap;
    std::map<std::string, std::vector<std::string>> m_employeeLocations;

    // ROS
    std::shared_ptr<PathPlannerNode> m_plannerNode;
    std::shared_ptr<ControllerNode> m_controllerNode;
    std::shared_ptr<ConfigNode> m_systemConfigNode;
    std::shared_ptr<MetricsNode> m_metricsNode;
    std::thread m_rosThread;
    std::thread m_simThread;

    std::shared_ptr<RosObserver> m_rosObserver;
};
