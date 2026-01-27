#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "../observer/observer.h"
#include "../util/types.h"

class TerminalView : public IObserver {
    // Colors removed as they are not standard in RCLCPP logging

public:
    std::string getName() override {
        return "Terminal";
    }

    void onEvent(int time, des::EventType type, const std::string& message) override {
        RCLCPP_INFO(rclcpp::get_logger("TerminalView"), "[%s] %s", des::toHumanReadableTime(time).c_str(), message.c_str());
    }

    void onRobotMoved(int time, const std::string& location, double /*distance*/) override {
        RCLCPP_INFO(rclcpp::get_logger("TerminalView"), "[%s] %s", des::toHumanReadableTime(time).c_str(), location.c_str());
    };
};
