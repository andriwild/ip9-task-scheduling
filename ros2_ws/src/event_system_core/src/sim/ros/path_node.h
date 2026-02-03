// How to write action clients: https://automaticaddison.com/how-to-create-an-action-ros-2-jazzy/
#pragma once

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "../../util/types.h"

struct SimplePose {
    double x, y, yaw;
};

struct PathResult {
    bool success;
    double distance;
};

using Cache = std::map<std::pair<std::string, std::string>, double>;

class PathPlannerNode : public rclcpp::Node {
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    PathPlannerNode(std::map<std::string, des::Point> locationMap):
        Node("event_system_planner_node"),
        m_locationMap(std::move(locationMap)) 
    {

        m_cache = {};
        m_client = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");

        RCLCPP_INFO(this->get_logger(), "Waiting for planner server...");
        m_ready = m_client->wait_for_action_server(std::chrono::seconds(5));
        if (m_ready) {
            RCLCPP_INFO(this->get_logger(), "Planner server ready!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planner server not available!");
        }
    }

    void clearCache() {
        m_cache.clear();
        RCLCPP_INFO(this->get_logger(), "Cache cleared");
    }

    PathResult serviceRequest(const SimplePose& goal, const SimplePose& start) {
        PathResult result{false, 0.0};
        m_resultReady = false;
        if (!m_ready) {
            return result;
        }

        auto goal_msg = ComputePathToPose::Goal();
        goal_msg.start      = toPose(start);
        goal_msg.goal       = toPose(goal);
        goal_msg.planner_id = "GridBased";
        goal_msg.use_start  = true;

        auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

        send_goal_options.result_callback =
            [this](const GoalHandle::WrappedResult& wrapped) {
                std::lock_guard lock(m_mutex);
                if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    m_currentResult.success = true;
                    m_currentResult.distance = calcDistance(wrapped.result->path);
                }
                m_resultReady = true;
                m_cv.notify_one();
            };

        m_client->async_send_goal(goal_msg, send_goal_options);

        std::unique_lock lock(m_mutex);
        if (m_cv.wait_for(lock, std::chrono::seconds(15), [this] { return m_resultReady; })) {
            result = m_currentResult;
        }

        return result;
    }

    bool isReady() const { return m_ready; }

    std::optional<double> calcDistance(const std::string& from, const std::string& to, bool useCache) {
        if(useCache) {
            auto it = m_cache.find({from, to});
            if(it != m_cache.end()) {
                RCLCPP_DEBUG(this->get_logger(), "Cache Hit");
                return it->second;
            }
        }

        auto fromIt = m_locationMap.find(from);
        auto toIt   = m_locationMap.find(to);

        if (fromIt == m_locationMap.end() || toIt == m_locationMap.end()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR\t%s or %s not found in map!", from.c_str(), to.c_str());
            for (auto [k, _] : m_locationMap) {
                RCLCPP_INFO(this->get_logger(), "%s", k.c_str());
            }
            return std::nullopt;
        }

        SimplePose start{fromIt->second.m_x, fromIt->second.m_y, 0.0};
        SimplePose goal{toIt->second.m_x, toIt->second.m_y, 0.0};

        auto result = serviceRequest(start, goal);

        if (!result.success) {
            return std::nullopt;
        }

        m_cache[{from, to}] = result.distance;
        return result.distance;
    }

private:
    geometry_msgs::msg::PoseStamped toPose(const SimplePose& p) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp    = this->now();
        msg.pose.position.x = p.x;
        msg.pose.position.y = p.y;
        msg.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, p.yaw);

        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        return msg;
    }

    double calcDistance(const nav_msgs::msg::Path& path) {
        double d = 0;
        for (size_t i = 1; i < path.poses.size(); ++i) {
            auto& p1 = path.poses[i - 1].pose.position;
            auto& p2 = path.poses[i].pose.position;
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            d += std::sqrt(dx * dx + dy * dy);
        }
        return d;
    }

    Cache m_cache;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr m_client;
    bool m_ready{false};

    std::mutex m_mutex;
    std::condition_variable m_cv;
    bool m_resultReady{false};
    PathResult m_currentResult{false, 0.0};
    std::map<std::string, des::Point> m_locationMap;
};
