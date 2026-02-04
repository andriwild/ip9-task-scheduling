#pragma once

#include <rclcpp/rclcpp.hpp>

#include "observer.h"

#include "event_system_msgs/msg/timeline_event.hpp"
#include "event_system_msgs/msg/timeline_move.hpp"
#include "event_system_msgs/msg/timeline_state_change.hpp"
#include "event_system_msgs/msg/timeline_meeting.hpp"
#include "event_system_msgs/msg/timeline_reset.hpp"
#include "event_system_msgs/msg/timeline_battery.hpp"

class RosObserver : public IObserver {
public:
    explicit RosObserver(rclcpp::Node::SharedPtr node) : m_node(node) {
        m_pubMove        = m_node->create_publisher<event_system_msgs::msg::TimelineMove>("/timeline/move"               , rclcpp::QoS(100));
        m_pubStateChange = m_node->create_publisher<event_system_msgs::msg::TimelineStateChange>("/timeline/state_change", rclcpp::QoS(100));
        m_pubReset       = m_node->create_publisher<event_system_msgs::msg::TimelineReset>("/timeline/reset"             , rclcpp::QoS(100));
        m_pubMeeting     = m_node->create_publisher<event_system_msgs::msg::TimelineMeeting>("/timeline/meeting"         , rclcpp::QoS(100));
        m_pubEvent       = m_node->create_publisher<event_system_msgs::msg::TimelineEvent>("/timeline/event"               , rclcpp::QoS(100));
        m_pubBattery     = m_node->create_publisher<event_system_msgs::msg::TimelineBattery>("/timeline/battery"             , rclcpp::QoS(100));
    }

    std::string getName() override {
        return "ROS";
    }

    void onEvent(int time, des::EventType type, const std::string& message) override { 
        auto msg = event_system_msgs::msg::TimelineEvent();
        msg.time = time;
        msg.type = static_cast<int>(type);
        msg.label = message;
        m_pubEvent->publish(msg);
    };

    void onRobotMoved(int time, const std::string& location, double /*distance*/) override {
        auto msg = event_system_msgs::msg::TimelineMove();
        msg.appointment_time = time;
        msg.label            = location;
        m_pubMove->publish(msg);
    }

    void onStateChanged(int time, const des::RobotStateType& type) override {
        auto msg = event_system_msgs::msg::TimelineStateChange();
        msg.appointment_time = time;
        msg.state            = static_cast<int>(type);
        m_pubStateChange->publish(msg);
    }

    void publishMeeting(std::shared_ptr<des::Appointment> appt, int startTime) {
        auto msg = event_system_msgs::msg::TimelineMeeting();
        msg.start_time       = startTime;
        msg.id               = appt->id;
        msg.appointment_time = appt->appointmentTime;
        msg.mission_state    = appt->state;
        msg.person_name      = appt->personName;
        msg.description      = appt->description;
        m_pubMeeting->publish(msg);
    }

    void publishReset() {
        auto msg = event_system_msgs::msg::TimelineReset();
        m_pubReset->publish(msg);
    }

    void publishBatteryState(int time, double soc, double balance) {
        auto msg = event_system_msgs::msg::TimelineBattery();
        msg.time = time;
        msg.soc = soc;
        msg.balance = balance;
        m_pubBattery->publish(msg);
    }

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<event_system_msgs::msg::TimelineMove>::SharedPtr m_pubMove;
    rclcpp::Publisher<event_system_msgs::msg::TimelineStateChange>::SharedPtr m_pubStateChange;
    rclcpp::Publisher<event_system_msgs::msg::TimelineReset>::SharedPtr m_pubReset;
    rclcpp::Publisher<event_system_msgs::msg::TimelineMeeting>::SharedPtr m_pubMeeting;
    rclcpp::Publisher<event_system_msgs::msg::TimelineEvent>::SharedPtr m_pubEvent;
    rclcpp::Publisher<event_system_msgs::msg::TimelineBattery>::SharedPtr m_pubBattery;
};
