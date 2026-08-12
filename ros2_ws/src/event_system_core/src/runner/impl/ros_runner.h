/*
 * Everything an app runner needs from ROS: the executor thread that
 * serves the nodes and the Nav2-backed path planner.
 * Runners that only replay the distance matrix do not need this.
 *
 */

#pragma once

#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "runner/runner.h"
#include "sim/ros/path_node.h"

namespace des {

class RosRunner : public IAppRunner {
protected:
    std::shared_ptr<PathPlannerNode> m_plannerNode;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::thread m_rosThread;

    std::shared_ptr<IPathPlanner> createNav2Planner() override {
        m_plannerNode = std::make_shared<PathPlannerNode>(m_rooms);
        return m_plannerNode;
    }

    void initROS(const std::vector<std::shared_ptr<rclcpp::Node>>& nodes) {
        // leads to spam messages on lower logger level
        rclcpp::get_logger("event_system_planner_node.rclcpp_action").set_level(rclcpp::Logger::Level::Warn);

        if (m_plannerNode && !m_plannerNode->isReady()) {
            throw std::runtime_error("Nav2 Planner initialization failed");
        }

        m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
        m_rosThread = std::thread([this, nodes] {
            for (const auto& node : nodes) {
                m_executor->add_node(node);
            }
            m_executor->spin();
            for (const auto& node : nodes) {
                m_executor->remove_node(node);
            }
        });
        DES_LOG_INFO("des.runner", "Launched all ROS Nodes");
    }

    void stopROS() {
        if (m_executor) {
            m_executor->cancel();
        }
        if (m_rosThread.joinable()) {
            m_rosThread.join();
        }
    }
};

}  // namespace des
