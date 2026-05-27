#pragma once

#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "observer.h"
#include "../plugins/order_registry.h"

#include "event_system_msgs/msg/timeline_event.hpp"
#include "event_system_msgs/msg/timeline_state_change.hpp"
#include "event_system_msgs/msg/timeline_meeting.hpp"
#include "event_system_msgs/msg/timeline_reset.hpp"

class RosObserver final : public IObserver {
public:
    explicit RosObserver(rclcpp::Node::SharedPtr node) : m_node(std::move(node)) {
        auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();
        m_pubStateChange = m_node->create_publisher<event_system_msgs::msg::TimelineStateChange>("/timeline/state_change", qos);
        m_pubReset       = m_node->create_publisher<event_system_msgs::msg::TimelineReset>("/timeline/reset"             , qos);
        m_pubMeeting     = m_node->create_publisher<event_system_msgs::msg::TimelineMeeting>("/timeline/meeting"         , qos);
        m_pubEvent       = m_node->create_publisher<event_system_msgs::msg::TimelineEvent>("/timeline/event"             , qos);
    }

    std::string getName() override {
        return "ROS";
    }

    void onEvent(const int time, des::EventType type, const std::string& message, const bool isDriving, const bool isCharging, const std::string& color = "", const int missionId = -1) override {
        auto msg = event_system_msgs::msg::TimelineEvent();
        msg.time = time;
        msg.type = static_cast<int>(type);
        msg.label = message;
        msg.is_driving = isDriving;
        msg.is_charging = isCharging;
        msg.color = color;
        msg.mission_id = missionId;
        m_pubEvent->publish(msg);
    };

    void onStateChanged(const int time, const des::RobotStateType& type, const std::string& name, const des::BatteryProps batStats) override {
        auto msg = event_system_msgs::msg::TimelineStateChange();
        msg.time          = time;
        msg.state         = static_cast<int>(type);
        msg.name          = name;
        msg.soc           = batStats.soc;
        msg.capacity      = batStats.capacity;
        msg.low_threshold = batStats.lowThreshold;
        m_pubStateChange->publish(msg);
    }

    void publishReset() const {
        const auto msg = event_system_msgs::msg::TimelineReset();
        m_pubReset->publish(msg);
    }

    // Event-driven: every time a mission becomes visible (arrival / dispatch),
    // ask the matching plugin to emit a TimelineMeeting. This replaces the old
    // up-front scan of the initial event queue.
    void onMissionPublished(const des::OrderPtr& order, int time) override {
        if (!order) return;
        auto& plugin = OrderRegistry::instance().get(order->type);
        plugin.publishTimeline(*order, time, *this);
    }

    void publishMeeting(const int id,
                        const int startTime,
                        const int scheduledTime,
                        const int state,
                        const std::string& orderType,
                        const std::string& personName,
                        const std::string& roomName,
                        const std::string& description,
                        const int executionMode) {
        auto msg = event_system_msgs::msg::TimelineMeeting();
        msg.start_time       = startTime;
        msg.id               = id;
        msg.appointment_time = scheduledTime;
        msg.mission_state    = state;
        msg.order_type       = orderType;
        msg.person_name      = personName;
        msg.room_name        = roomName;
        msg.description      = description;
        msg.execution_mode   = static_cast<uint8_t>(executionMode);
        m_pubMeeting->publish(msg);
    }

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<event_system_msgs::msg::TimelineStateChange>::SharedPtr m_pubStateChange;
    rclcpp::Publisher<event_system_msgs::msg::TimelineReset>::SharedPtr m_pubReset;
    rclcpp::Publisher<event_system_msgs::msg::TimelineMeeting>::SharedPtr m_pubMeeting;
    rclcpp::Publisher<event_system_msgs::msg::TimelineEvent>::SharedPtr m_pubEvent;
};
