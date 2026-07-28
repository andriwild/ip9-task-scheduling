/**
 * @file robot_marker.h
 * @brief Implements observer interface and publishes the robots location on a ROS topic.
 * 
 */

#pragma once

#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "observer.h"
#include "../util/types.h"

class RobotMarkerObserver final : public IObserver {
public:
    RobotMarkerObserver(const rclcpp::Node::SharedPtr& node, des::LocationMap locationMap)
        : m_locationMap(std::move(locationMap))
    {
        auto qos = rclcpp::QoS(1).transient_local();
        m_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", qos);
    }

    std::string getName() override {
        return "RobotMarker";
    }

    void onRobotMoved(int /*time*/, const std::string& location, double /*distance*/) override {
        const auto it = m_locationMap.find(location);
        if (it == m_locationMap.end()) {
            return;
        }
        m_position = it->second.m_p;
        publish();
    }

    void onRobotMovedTo(int /*time*/, const des::Point& position, double /*distance*/ = 0.0) override {
        m_position = position;
        publish();
    }

private:
    void publish() {
        // publish yellow robot marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "robot";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = m_position.m_x;
        marker.pose.position.y = m_position.m_y;
        marker.pose.position.z = 0.25;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        visualization_msgs::msg::MarkerArray markers;
        markers.markers.push_back(marker);
        m_publisher->publish(markers);
    }

    des::Point m_position;
    des::LocationMap m_locationMap;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_publisher;
};
