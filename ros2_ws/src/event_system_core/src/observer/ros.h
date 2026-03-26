#pragma once

#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "observer.h"

#include "event_system_msgs/msg/timeline_event.hpp"
#include "event_system_msgs/msg/timeline_state_change.hpp"
#include "event_system_msgs/msg/timeline_meeting.hpp"
#include "event_system_msgs/msg/timeline_reset.hpp"

class RosObserver final : public IObserver {
public:
    explicit RosObserver(rclcpp::Node::SharedPtr node) : m_node(std::move(node)) {
        m_pubStateChange = m_node->create_publisher<event_system_msgs::msg::TimelineStateChange>("/timeline/state_change", rclcpp::QoS(500));
        m_pubReset       = m_node->create_publisher<event_system_msgs::msg::TimelineReset>("/timeline/reset"             , rclcpp::QoS(10));
        m_pubMeeting     = m_node->create_publisher<event_system_msgs::msg::TimelineMeeting>("/timeline/meeting"         , rclcpp::QoS(100));
        m_pubEvent       = m_node->create_publisher<event_system_msgs::msg::TimelineEvent>("/timeline/event"             , rclcpp::QoS(500));
    }

    std::string getName() override {
        return "ROS";
    }

    void onEvent(const int time, des::EventType type, const std::string& message, const bool isDriving, const bool isCharging, const std::string& color = "") override {
        auto msg = event_system_msgs::msg::TimelineEvent();
        msg.time = time;
        msg.type = static_cast<int>(type);
        msg.label = message;
        msg.is_driving = isDriving;
        msg.is_charging = isCharging;
        msg.color = color;
        m_pubEvent->publish(msg);
    };

    void onStateChanged(const int time, const des::RobotStateType& type, const des::BatteryProps batStats) override {
        auto msg = event_system_msgs::msg::TimelineStateChange();
        msg.time          = time;
        msg.state         = static_cast<int>(type);
        msg.soc           = batStats.soc;
        msg.capacity      = batStats.capacity;
        msg.low_threshold = batStats.lowThreshold;
        m_pubStateChange->publish(msg);
    }

    void publishReset() const {
        const auto msg = event_system_msgs::msg::TimelineReset();
        m_pubReset->publish(msg);
    }

    void publishMeeting(const std::shared_ptr<des::Appointment>& appt, const int startTime) {
        auto msg = event_system_msgs::msg::TimelineMeeting();
        msg.start_time       = startTime;
        msg.id               = appt->id;
        msg.appointment_time = appt->appointmentTime;
        msg.mission_state    = appt->state;
        msg.person_name      = appt->personName;
        msg.description      = appt->description;
        m_pubMeeting->publish(msg);
    }

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<event_system_msgs::msg::TimelineStateChange>::SharedPtr m_pubStateChange;
    rclcpp::Publisher<event_system_msgs::msg::TimelineReset>::SharedPtr m_pubReset;
    rclcpp::Publisher<event_system_msgs::msg::TimelineMeeting>::SharedPtr m_pubMeeting;
    rclcpp::Publisher<event_system_msgs::msg::TimelineEvent>::SharedPtr m_pubEvent;
};
