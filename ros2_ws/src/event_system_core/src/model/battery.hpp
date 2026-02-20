#pragma once

#include "../util/types.h"
#include <rclcpp/rclcpp.hpp>

class Battery {
    int m_lastBalanceUpdate = 0;

    double m_designCapacity;      // Ah - battery design capacity
    double m_currentCapacity;     // Ah
    double m_initialCapacity;     // Ah - on simulation start
    double m_lowBatteryThreshold; // %
    double m_fullBatteryThreshold; // %
    double m_voltage = 12.0;
    rclcpp::Logger m_logger;

public:
    explicit Battery(
        const double capacity,
        const double initialCapacity,
        const double lowBatteryThreshold,
        const double fullBatteryThreshold,
        const rclcpp::Logger &logger
    )
        : m_designCapacity(capacity)
        , m_initialCapacity(initialCapacity)
        , m_lowBatteryThreshold(lowBatteryThreshold)
        , m_fullBatteryThreshold(fullBatteryThreshold)
        , m_logger(logger)
    {
        m_currentCapacity = initialCapacity;
    }

    void updateConfig(const double designCapacity, const double initialCapacity, const double lowThreshold, const double fullThreshold) {
        m_designCapacity       = designCapacity;
        m_initialCapacity      = initialCapacity;
        m_lowBatteryThreshold  = lowThreshold;
        m_fullBatteryThreshold = fullThreshold;
        RCLCPP_INFO(rclcpp::get_logger("Battery"), "Config updated");
    }

    void updateBalance(const int time, const double energyConsumption) {
        // energy in Watt, time in seconds (+ discharge, - charge)
        // Ah = (W * s) / (3600 * V)
        const int timeDelta = time - m_lastBalanceUpdate;
        m_lastBalanceUpdate = time;
        m_currentCapacity -= energyConsumption * timeDelta / (3600 * m_voltage);

        if (m_currentCapacity < m_lowBatteryThreshold / 100 * m_designCapacity) {
            RCLCPP_WARN(rclcpp::get_logger("Battery"), "Batter Low - SOC: %.1f", m_currentCapacity / m_designCapacity);
        }

        if (m_currentCapacity <= 0) {
            RCLCPP_ERROR(rclcpp::get_logger("Battery"), "Battery discharged - no energy left");
        }
        m_currentCapacity = clip(m_currentCapacity, 0.0, m_designCapacity);
    }

    void reset(const int startTime) {
        m_lastBalanceUpdate = startTime;
        m_currentCapacity = m_initialCapacity;
        RCLCPP_INFO(rclcpp::get_logger("Battery"), "Reset: initial capactiy: %.1f", m_initialCapacity);
    }

    des::BatteryProps getStats() const {
        return { m_currentCapacity / m_designCapacity, m_designCapacity, m_lowBatteryThreshold };
    }

    bool isBatteryLow() const { return m_currentCapacity < m_lowBatteryThreshold / 100 * m_designCapacity; }

    bool isBatteryFull() const { return m_currentCapacity > m_fullBatteryThreshold / 100 * m_designCapacity; }

    double timeToFull(const double chargingPowerWatts) const {
        if (chargingPowerWatts <= 0) return -1.0;

        const double fullCapacity = m_fullBatteryThreshold / 100.0 * m_designCapacity;
        if (m_currentCapacity >= fullCapacity) return 0.0;

        const double capacityDiff = fullCapacity - m_currentCapacity; // Ah
        // Ah = (W * s) / (3600 * V) => s = (Ah * 3600 * V) / W
        return (capacityDiff * 3600.0 * m_voltage) / chargingPowerWatts;
    }

private:
    template<typename T>
    T clip(const T &n, const T &lower, const T &upper) {
        return std::max(lower, std::min(n, upper));
    }
};
