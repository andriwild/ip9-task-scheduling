#pragma once

#include <rclcpp/rclcpp.hpp>

#include "event_system_msgs/msg/timeline_event.hpp"
#include "observer.h"

class RosObserver : public IObserver {
public:
    explicit RosObserver(rclcpp::Node::SharedPtr node) : m_node(node) {
        m_publisher = m_node->create_publisher<event_system_msgs::msg::TimelineEvent>(
            "/timeline_events",
            rclcpp::QoS(100));
    }

    std::string getName() override {
        return "ROS";
    }

    void onLog(int /*time*/, const std::string& /*message*/) override {
        // auto msg = event_system_msgs::msg::TimelineEvent();
        // msg.time = time;
        // msg.type = event_system_msgs::msg::TimelineEvent::LOG;
        // msg.label = message;
        // m_publisher->publish(msg);
    }

    void onRobotMoved(int time, const std::string& location, double /*distance*/) override {
        auto msg = event_system_msgs::msg::TimelineEvent();
        msg.time = time;
        msg.type = event_system_msgs::msg::TimelineEvent::MOVE;
        msg.label = location;
        m_publisher->publish(msg);
    }

    void onStateChanged(int time, const RobotStateType& type) override {
        auto msg = event_system_msgs::msg::TimelineEvent();
        msg.time = time;
        msg.type = event_system_msgs::msg::TimelineEvent::STATE_CHANGE;
        msg.state = static_cast<int>(type);
        m_publisher->publish(msg);
    }

    void publishMeeting(std::shared_ptr<des::Appointment> appt, int startTime) {
        auto msg = event_system_msgs::msg::TimelineEvent();
        msg.time = startTime;  // Using startTime as the event time
        msg.type = event_system_msgs::msg::TimelineEvent::MEETING;
        msg.duration = appt->appointmentTime - startTime;  // Assuming duration is diff
        msg.description = appt->description;
        msg.person_name = appt->personName;
        msg.duration = appt->appointmentTime - startTime;
        m_publisher->publish(msg);
    }

    void publishReset() {
        auto msg = event_system_msgs::msg::TimelineEvent();
        msg.type = event_system_msgs::msg::TimelineEvent::RESET;
        m_publisher->publish(msg);
    }

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<event_system_msgs::msg::TimelineEvent>::SharedPtr m_publisher;
};
