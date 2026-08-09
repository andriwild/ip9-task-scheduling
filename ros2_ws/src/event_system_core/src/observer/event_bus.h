#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "../util/log.h"
#include "observer.h"

namespace des {

class EventBus {
    std::vector<std::shared_ptr<IObserver>> m_observers;

public:
    void addObserver(const std::shared_ptr<IObserver>& observer) {
        DES_LOG_DEBUG("des.event_bus", "Observer added: %s", observer->getName().c_str());
        m_observers.emplace_back(observer);
    }

    void removeObserver(const std::shared_ptr<IObserver>& observer) {
        auto it = std::remove(m_observers.begin(), m_observers.end(), observer);
        m_observers.erase(it, m_observers.end());
    }

    void notifyEvent(int time, const EventType& type, const std::string& name,
                     bool isDriving, bool isCharging, const std::string& color = "",
                     int missionId = -1) const {
        for (const auto& obs : m_observers) {
            obs->onEvent(time, type, name, isDriving, isCharging, color, missionId);
        }
    }

    void notifyStateChanged(int time, RobotStateType newState, const std::string& name, BatteryProps batStats) const {
        for (const auto& obs : m_observers) {
            obs->onStateChanged(time, newState, name, batStats);
        }
    }

    void notifyMissionPublished(const OrderPtr& order, int time) const {
        for (const auto& obs : m_observers) {
            obs->onMissionPublished(order, time);
        }
    }
};

}  // namespace des
