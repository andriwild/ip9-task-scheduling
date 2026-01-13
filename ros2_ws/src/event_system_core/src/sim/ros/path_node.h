// How to write action clients: https://automaticaddison.com/how-to-create-an-action-ros-2-jazzy/
#pragma once

#include <chrono>
#include <mutex>
#include <cmath>

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include "../../util/types.h"

struct SimplePose {
    double x, y, yaw;
};

struct PathResult {
    bool success;
    double distance;
};

class PathPlannerNode : public rclcpp::Node {
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    PathPlannerNode(std::map<std::string, des::Point> locationMap) :
        Node("des_path_planner_node"),
        locationMap(locationMap)
    {
        m_client = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");

        RCLCPP_INFO(this->get_logger(), "Waiting for planner server...");
        m_ready = m_client->wait_for_action_server(std::chrono::seconds(5));
        if (m_ready) {
            RCLCPP_INFO(this->get_logger(), "Planner server ready!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planner server not available!");
        }
    }

    PathResult computeDistance(const SimplePose& goal) {
        auto t1 = std::chrono::high_resolution_clock::now();
        auto r = computeDistance(goal, {}, false);
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::cout << "Duration: " << duration.count() << " ms" << std::endl;
        return r;
    }

    PathResult computeDistance(const SimplePose& start, const SimplePose& goal) {
        auto t1 = std::chrono::high_resolution_clock::now();
        auto r = computeDistance(goal, start, true);
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::cout << "Duration: " << duration.count() << " ms" << std::endl;
        return r;
    }

    PathResult computeDistance(const SimplePose& goal, const SimplePose& start, bool use_start) {
        PathResult result{false, 0.0};
        m_resultReady = false;
        if (!m_ready) return result;

        auto goal_msg = ComputePathToPose::Goal();
        goal_msg.start = toPose(start);
        goal_msg.goal = toPose(goal);
        goal_msg.planner_id = "GridBased";
        goal_msg.use_start = use_start;

        auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

        send_goal_options.result_callback =
            [this](const GoalHandle::WrappedResult & wrapped) {
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
        if (m_cv.wait_for(lock, std::chrono::seconds(15), [this]{ return m_resultReady; })) {
            result = m_currentResult;
        }

        return result;
    }

    bool isReady() const { return m_ready; }


    std::optional<double> estimateDistance(const std::string& from, const std::string& to) {
        auto fromIt = locationMap.find(from);
        auto toIt   = locationMap.find(to);

        if(fromIt == locationMap.end() || toIt == locationMap.end()){
            std::cerr << "ERROR\t" <<  from << " or " << to << "  not found in map!" << std::endl;
            for(auto [k,_]: locationMap) {
                std::cout << k << std::endl;
            }
            return std::nullopt;
        }

        SimplePose start { fromIt->second.m_x, fromIt->second.m_y, 0.0 };
        SimplePose goal  { toIt->second.m_x, toIt->second.m_y, 0.0 };
        
        auto result = computeDistance(start, goal);

        if (!result.success) return std::nullopt;

        return result.distance;
    }

private:
    geometry_msgs::msg::PoseStamped toPose(const SimplePose& p) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
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
            auto& p1 = path.poses[i-1].pose.position;
            auto& p2 = path.poses[i].pose.position;
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            d += std::sqrt(dx*dx + dy*dy);
        }
        return d;
    }

    rclcpp_action::Client<ComputePathToPose>::SharedPtr m_client;
    bool m_ready{false};

    std::mutex m_mutex;
    std::condition_variable m_cv;
    bool m_resultReady{false};
    PathResult m_currentResult{false, 0.0};
    std::map<std::string, des::Point> locationMap;
};
