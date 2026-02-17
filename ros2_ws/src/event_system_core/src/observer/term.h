#pragma once

#include <rclcpp/rclcpp.hpp>

#include "../observer/observer.h"
#include "../util/types.h"

class TerminalView final : public IObserver {

public:
    std::string getName() override { return "Terminal"; }

    void onEvent(const int time, des::EventType /*type*/, const std::string& message, const bool /*isDriving*/, const bool /*isCharging*/) override {
        RCLCPP_INFO(rclcpp::get_logger("TerminalView"), "[%s] %s", des::toHumanReadableTime(time).c_str(), message.c_str());
    }

    void onRobotMoved(const int time, const std::string& location, double /*distance*/) override {
        RCLCPP_INFO(rclcpp::get_logger("TerminalView"), "[%s] %s", des::toHumanReadableTime(time).c_str(), location.c_str());
    };
};
