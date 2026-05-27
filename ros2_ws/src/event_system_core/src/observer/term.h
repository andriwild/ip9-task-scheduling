#pragma once

#include <rclcpp/rclcpp.hpp>

#include "../util/log.h"
#include "../observer/observer.h"
#include "../util/types.h"

class TerminalView final : public IObserver {

public:
    std::string getName() override { return "Terminal"; }

    void onEvent(const int time, des::EventType /*type*/, const std::string& message, const bool /*isDriving*/, const bool /*isCharging*/, const std::string& /*color*/ = "", const int /*missionId*/ = -1) override {
        DES_LOG_INFO(rclcpp::get_logger("des.observer.term"), "[%s] %s", des::toHumanReadableTime(time).c_str(), message.c_str());
    }

    void onRobotMoved(const int time, const std::string& location, double /*distance*/) override {
        DES_LOG_INFO(rclcpp::get_logger("des.observer.term"), "[%s] %s", des::toHumanReadableTime(time).c_str(), location.c_str());
    };
};
