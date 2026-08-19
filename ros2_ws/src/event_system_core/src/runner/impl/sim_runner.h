#pragma once

#include <memory>
#include "../../util/log.h"
#include <thread>
#include <vector>

#include "engine/event.h"
#include "../../observer/ros.h"
#include "../../runner/runner.h"
#include "../../sim/ros/config.h"
#include "../../sim/ros/controller.h"
#include "../../sim/ros/path_node.h"
#include "../runner.h"
#include "ros_runner.h"
#include "model/sim_config.h"

namespace des {

class SimRunner final : public RosRunner {
public:

    explicit SimRunner() {
        m_rooms = loadRooms();

        createPlanner();
        m_controllerNode   = std::make_shared<ControllerNode>();
        m_systemConfigNode = std::make_shared<ConfigNode>();

        std::vector<std::shared_ptr<rclcpp::Node>> nodes = { m_controllerNode, m_systemConfigNode };
        if (m_plannerNode) nodes.push_back(m_plannerNode);
        initROS(nodes);
    }

    ~SimRunner() override { SimRunner::shutdown(); }

    static std::unique_ptr<IAppRunner> create(int argc, char* argv[]) {
        rclcpp::init(argc, argv);
        DES_LOG_INFO("des.runner", "\n----- Descrete Event Sytem -----");
        DES_LOG_INFO("des.runner", "C++ Version: %ld", __cplusplus);
        return std::make_unique<SimRunner>();
    }

    void setupApplication() override;
    void updateConfig() override;
    RunState loadAppState() const override;
    void enterPause() const override;
    void onSimulationComplete() override { enterPause(); }
    void reset() override;

    void shutdown() override {
        stopROS();

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
    void updateConfig(std::shared_ptr<SimConfig> config);

    std::shared_ptr<ControllerNode> m_controllerNode;
    std::shared_ptr<ConfigNode> m_systemConfigNode;
    std::thread m_simThread;
    std::shared_ptr<RosObserver> m_rosObserver;
};

}  // namespace des
