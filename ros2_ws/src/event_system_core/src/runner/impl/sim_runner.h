#pragma once

#include <memory>
#include <thread>
#include <vector>

#include "../../model/event.h"
#include "../../observer/metrics.h"
#include "../../observer/ros.h"
#include "../../runner/runner.h"
#include "../../sim/ros/config.h"
#include "../../sim/ros/controller.h"
#include "../../sim/ros/path_node.h"
#include "../runner.h"

class SimRunner final : public IAppRunner {
public:

    explicit SimRunner() {
        m_locationMap      = IAppRunner::loadPointsOfInterest({"wsr_user", "wsr_password"});

        m_plannerNode      = std::make_shared<PathPlannerNode>(m_locationMap);
        m_controllerNode   = std::make_shared<ControllerNode>();
        m_systemConfigNode = std::make_shared<ConfigNode>();
        m_metricsNode      = std::make_shared<MetricsNode>();

        IAppRunner::initROS({ m_plannerNode, m_controllerNode, m_systemConfigNode, m_metricsNode });
    }

    ~SimRunner() override { SimRunner::shutdown(); }

    static std::unique_ptr<IAppRunner> create(int argc, char* argv[]) {
        rclcpp::init(argc, argv);
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "\n----- Descrete Event Sytem -----");
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "C++ Version: %ld", __cplusplus);
        return std::make_unique<SimRunner>();
    }

    void setupApplication(const std::string& path) override;
    void updateConfig() override;
    int loadAppState() const override;
    void enterPause() const override;
    void reset() override;

    void shutdown() override {
        m_executor->cancel();

        if (m_rosThread.joinable()) {
            m_rosThread.join();
        }
        if (m_simThread.joinable()) {
            m_simThread.join();
        }

        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

private:
    void reloadSimulationData();
    void buildSimulation();
    void rebuildEventQueue();
    void updateConfig(std::shared_ptr<des::SimConfig> config);

    std::shared_ptr<ControllerNode> m_controllerNode;
    std::shared_ptr<ConfigNode> m_systemConfigNode;
    std::thread m_simThread;
    std::shared_ptr<RosObserver> m_rosObserver;
};
