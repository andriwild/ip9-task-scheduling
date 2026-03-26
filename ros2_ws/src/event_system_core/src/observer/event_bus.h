#pragma once

#include <algorithm>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "observer.h"

class EventBus {
    std::vector<std::shared_ptr<IObserver>> m_observers;

public:
    void addObserver(const std::shared_ptr<IObserver>& observer) {
        RCLCPP_INFO(rclcpp::get_logger("EventBus"), "Observer added: %s", observer->getName().c_str());
        m_observers.emplace_back(observer);
    }

    void removeObserver(const std::shared_ptr<IObserver>& observer) {
        auto it = std::remove(m_observers.begin(), m_observers.end(), observer);
        m_observers.erase(it, m_observers.end());
    }

    void clear() {
        m_observers.clear();
    }

    void notifyEvent(int time, const des::EventType& type, const std::string& name,
                     bool isDriving, bool isCharging, const std::string& color = "") const {
        for (const auto& obs : m_observers) {
            obs->onEvent(time, type, name, isDriving, isCharging, color);
        }
    }

    void notifyStateChanged(int time, des::RobotStateType newState, des::BatteryProps batStats) const {
        for (const auto& obs : m_observers) {
            obs->onStateChanged(time, newState, batStats);
        }
    }

    void notifyMissionComplete(int time, const des::MissionState& state, int timeDiff) const {
        for (const auto& obs : m_observers) {
            obs->onMissionComplete(time, state, timeDiff);
        }
    }

    void notifyMoved(int time, const std::string& location, double distance) const {
        for (const auto& obs : m_observers) {
            obs->onRobotMoved(time, location, distance);
        }
    }
};
