#pragma once

#include "../util/types.h"
#include <rclcpp/rclcpp.hpp>

#include "../util/log.h"
class Battery {
    int m_lastBalanceUpdate = 0;

    double m_designCapacity;      // Ah - battery design capacity
    double m_currentCapacity;     // Ah
    double m_initialCapacity;     // Ah - on simulation start
    double m_lowBatteryThreshold; // %
    double m_fullBatteryThreshold; // %

public:
    static constexpr double kVoltage = 12.0;
    explicit Battery(
        const double capacity,
        const double initialCapacity,
        const double lowBatteryThreshold,
        const double fullBatteryThreshold
    )
        : m_designCapacity(capacity)
        , m_initialCapacity(initialCapacity)
        , m_lowBatteryThreshold(lowBatteryThreshold)
        , m_fullBatteryThreshold(fullBatteryThreshold)
    {
        m_currentCapacity = initialCapacity;
    }

    void updateConfig(const double designCapacity, const double initialCapacity, const double lowThreshold, const double fullThreshold) {
        m_designCapacity       = designCapacity;
        m_initialCapacity      = initialCapacity;
        m_lowBatteryThreshold  = lowThreshold;
        m_fullBatteryThreshold = fullThreshold;
        DES_LOG_INFO(rclcpp::get_logger("des.battery"), "Config updated");
    }

    void updateBalance(const int time, const double energyConsumption) {
        // energy in Watt, time in seconds (+ discharge, - charge)
        // Ah = (W * s) / (3600 * V)
        const int timeDelta = time - m_lastBalanceUpdate;
        m_lastBalanceUpdate = time;
        const double capacityDiff = energyConsumption * timeDelta / (3600 * kVoltage);
        m_currentCapacity -= capacityDiff;
        
        //DES_LOG_DEBUG(rclcpp::get_logger("des.battery"), "updateBalance: timeDelta %ds, energyConsumption %.2fW, capacity updated by %.3fAh -> %.3fAh", timeDelta, energyConsumption, -capacityDiff, m_currentCapacity);

        if (m_currentCapacity < m_lowBatteryThreshold / 100 * m_designCapacity) {
            DES_LOG_WARN(rclcpp::get_logger("des.battery"), "Battery Low - SOC: %.1f", m_currentCapacity / m_designCapacity);
        }

        if (m_currentCapacity <= 0) {
            DES_LOG_ERROR(rclcpp::get_logger("des.battery"), "Battery discharged - no energy left");
        }
        m_currentCapacity = clip(m_currentCapacity, 0.0, m_designCapacity);
    }

    void reset(const int startTime) {
        m_lastBalanceUpdate = startTime;
        m_currentCapacity = m_initialCapacity;
        DES_LOG_INFO(rclcpp::get_logger("des.battery"), "Reset: initial capactiy: %.1f", m_initialCapacity);
    }

    des::BatteryProps getStats() const {
        return { m_currentCapacity / m_designCapacity, m_designCapacity, m_lowBatteryThreshold };
    }

    bool isBatteryLow() const { 
        const bool isLow = m_currentCapacity < m_lowBatteryThreshold / 100 * m_designCapacity;
        //DES_LOG_DEBUG(rclcpp::get_logger("des.battery"), "isBatteryLow: %d", isLow);
        return isLow;
    }

    bool isBatteryFull() const { 
        const bool isFull = m_currentCapacity > m_fullBatteryThreshold / 100 * m_designCapacity;
        DES_LOG_DEBUG(rclcpp::get_logger("des.battery"), "isBatteryFull: %d", isFull);
        return isFull;
    }

    double timeToFull(const double chargingPowerWatts) const {
        if (chargingPowerWatts <= 0) return -1.0;

        const double fullCapacity = m_fullBatteryThreshold / 100.0 * m_designCapacity;
        if (m_currentCapacity >= fullCapacity) return 0.0;

        const double capacityDiff = fullCapacity - m_currentCapacity; // Ah
        // Ah = (W * s) / (3600 * V) => s = (Ah * 3600 * V) / W
        const double duration = (capacityDiff * 3600.0 * kVoltage) / chargingPowerWatts;
        DES_LOG_DEBUG(rclcpp::get_logger("des.battery"), "Calculate time to full: %f", duration);
        return duration;
    }

private:
    template<typename T>
    T clip(const T &n, const T &lower, const T &upper) {
        return std::max(lower, std::min(n, upper));
    }
};
