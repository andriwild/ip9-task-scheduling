// How to write action clients: https://automaticaddison.com/how-to-create-an-action-ros-2-jazzy/
#pragma once

#include <mutex>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>

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

    PathPlannerNode() : Node("path_planner_node") {
        client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");

        RCLCPP_INFO(this->get_logger(), "Waiting for planner server...");
        ready_ = client_->wait_for_action_server(std::chrono::seconds(5));
        if (ready_) {
            RCLCPP_INFO(this->get_logger(), "Planner server ready!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planner server not available!");
        }
    }

    PathResult computeDistance(const SimplePose& goal) {
        return computeDistance(goal, {}, false);
    }

    PathResult computeDistance(const SimplePose& start, const SimplePose& goal) {
        return computeDistance(goal, start, true);
    }

    PathResult computeDistance(const SimplePose& goal, const SimplePose& start, bool use_start) {
        PathResult result{false, 0.0};
        result_ready_ = false;
        if (!ready_) return result;

        auto goal_msg = ComputePathToPose::Goal();
        goal_msg.start = toPose(start);
        goal_msg.goal = toPose(goal);
        goal_msg.planner_id = "GridBased";
        goal_msg.use_start = use_start;

        auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

        send_goal_options.result_callback =
            [this](const GoalHandle::WrappedResult & wrapped) {
                std::lock_guard lock(mutex_);
                if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    current_result_.success = true;
                    current_result_.distance = calcDistance(wrapped.result->path);
                }
                result_ready_ = true;
                cv_.notify_one();
            };

        client_->async_send_goal(goal_msg, send_goal_options);

        std::unique_lock lock(mutex_);
        if (cv_.wait_for(lock, std::chrono::seconds(15), [this]{ return result_ready_; })) {
            result = current_result_;
        }

        return result;
    }

    bool isReady() const { return ready_; }

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

    rclcpp_action::Client<ComputePathToPose>::SharedPtr client_;
    bool ready_{false};

    std::mutex mutex_;
    std::condition_variable cv_;
    bool result_ready_{false};
    PathResult current_result_{false, 0.0};
};
