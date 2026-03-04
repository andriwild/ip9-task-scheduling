#pragma once

#include <memory>
#include <thread>
#include <vector>

#include "../../model/context.h"
#include "../../model/event.h"
#include "../../observer/metrics.h"
#include "../../observer/ros.h"
#include "../../runner/runner.h"
#include "../../sim/ros/config.h"
#include "../../sim/ros/controller.h"
#include "../../sim/ros/path_node.h"
#include "../../sim/scheduler.h"
#include "../runner.h"

class SimRunner final : public IAppRunner {
public:

    SimRunner() = default;


    ~SimRunner() override {
        if (m_rosThread.joinable()) {
            m_rosThread.join();
        }
        if (m_simThread.joinable()) {
            m_simThread.join();
        }
    }

    static std::unique_ptr<IAppRunner> create(int argc, char* argv[]) {
        rclcpp::init(argc, argv);
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "\n----- Descrete Event Sytem -----");
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "C++ Version: %ld", __cplusplus);
        return std::make_unique<SimRunner>();
    }

    void setupApplication() override;
    void updateConfig() override;
    int loadAppState() const override;
    void enterPause() const override;
    void reset() override;

    void shutdown() override {
        m_executor.cancel();
        rclcpp::shutdown();
    }

private:
    void initROS();
    void setupObservers(bool headless, bool verbose);
    void updateConfig(std::shared_ptr<des::SimConfig> config);
    void setupQueue(const std::shared_ptr<des::SimConfig> &config);

    std::unique_ptr<Scheduler> m_scheduler;
    std::vector<std::shared_ptr<des::Appointment>> m_appointments;
    std::shared_ptr<des::SimConfig> m_config;
    std::map<std::string, des::Point> m_locationMap;
    std::map<std::string, std::vector<std::string>> m_employeeLocations;

    // ROS
    rclcpp::executors::MultiThreadedExecutor m_executor;
    std::shared_ptr<PathPlannerNode> m_plannerNode;
    std::shared_ptr<ControllerNode> m_controllerNode;
    std::shared_ptr<ConfigNode> m_systemConfigNode;
    std::shared_ptr<MetricsNode> m_metricsNode;
    std::thread m_rosThread;
    std::thread m_simThread;

    std::shared_ptr<RosObserver> m_rosObserver;
};
