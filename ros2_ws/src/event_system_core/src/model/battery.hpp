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

    double m_voltage = 12.0;
    double m_cvThreshold   = 0.8;
    double m_taperFraction = 0.5;
    bool m_chargeToFull    = true;

    double capacityToTime(const double capacityDiff, const double powerWatts) const {
        return capacityDiff * 3600.0 * m_voltage / powerWatts;
    }

    double targetCapacity() const {
        const double cvCap   = m_cvThreshold * m_designCapacity;
        const double fullCap = m_fullBatteryThreshold / 100.0 * m_designCapacity;
        return m_chargeToFull ? fullCap : cvCap;
    }

public:
    explicit Battery(
        const double capacity,
        const double initialCapacity,
        const double lowBatteryThreshold,
        const double fullBatteryThreshold,
        const double voltage = 12.0,
        const double cvThreshold = 0.8,
        const double taperFraction = 0.5,
        const bool chargeToFull = true
    )
        : m_designCapacity(capacity)
        , m_initialCapacity(initialCapacity)
        , m_lowBatteryThreshold(lowBatteryThreshold)
        , m_fullBatteryThreshold(fullBatteryThreshold)
        , m_voltage(voltage)
        , m_cvThreshold(cvThreshold)
        , m_taperFraction(taperFraction)
        , m_chargeToFull(chargeToFull)
    {
        m_currentCapacity = initialCapacity;
    }

    void updateConfig(const double designCapacity, const double initialCapacity, const double lowThreshold, const double fullThreshold,
                      const double voltage, const double cvThreshold, const double taperFraction, const bool chargeToFull) {
        m_designCapacity       = designCapacity;
        m_initialCapacity      = initialCapacity;
        m_lowBatteryThreshold  = lowThreshold;
        m_fullBatteryThreshold = fullThreshold;
        m_voltage              = voltage;
        m_cvThreshold          = cvThreshold;
        m_taperFraction        = taperFraction;
        m_chargeToFull         = chargeToFull;
        DES_LOG_INFO(rclcpp::get_logger("des.battery"), "Config updated");
    }

    void updateBalance(const int time, const double energyConsumption) {
        // energy in Watt, time in seconds (+ discharge, - charge)
        // Ah = (W * s) / (3600 * V)
        const int timeDelta = time - m_lastBalanceUpdate;
        m_lastBalanceUpdate = time;
        const double capacityDiff = energyConsumption * timeDelta / (3600 * m_voltage);
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

    void completeCharge() {
        m_currentCapacity = targetCapacity();
    }

    void reset(const int startTime) {
        m_lastBalanceUpdate = startTime;
        m_currentCapacity = m_initialCapacity;
        DES_LOG_INFO(rclcpp::get_logger("des.battery"), "Reset: initial capactiy: %.1f", m_initialCapacity);
    }

    des::BatteryProps getStats() const {
        return { m_currentCapacity / m_designCapacity, m_designCapacity, m_lowBatteryThreshold };
    }

    double getVoltage() const { return m_voltage; }

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

    double timeToFull(const double phaseOnePowerWatts) const {
        if (phaseOnePowerWatts <= 0) return -1.0;

        const double targetCap = targetCapacity();
        if (m_currentCapacity >= targetCap) return 0.0;

        const double cvCap = m_cvThreshold * m_designCapacity;
        double duration = 0.0;

        const double phaseOneEnd = std::min(targetCap, cvCap);
        if (m_currentCapacity < phaseOneEnd) {
            duration += capacityToTime(phaseOneEnd - m_currentCapacity, phaseOnePowerWatts);
        }
        if (targetCap > cvCap) {
            const double phaseTwoStart = std::max(m_currentCapacity, cvCap);
            duration += capacityToTime(targetCap - phaseTwoStart, phaseOnePowerWatts * m_taperFraction);
        }
        DES_LOG_DEBUG(rclcpp::get_logger("des.battery"), "Calculate time to full: %f", duration);
        return duration;
    }

    double timeToPhaseTransition(const double phaseOnePowerWatts) const {
        if (phaseOnePowerWatts <= 0) return -1.0;
        if (!m_chargeToFull) return -1.0;

        const double cvCap   = m_cvThreshold * m_designCapacity;
        const double fullCap = m_fullBatteryThreshold / 100.0 * m_designCapacity;
        if (fullCap <= cvCap) return -1.0;
        if (m_currentCapacity >= cvCap) return -1.0;

        return capacityToTime(cvCap - m_currentCapacity, phaseOnePowerWatts);
    }

    double chargingConsumption(const double chargingRate, const double baseConsumption) const {
        const double netFull = chargingRate - baseConsumption;
        const double cvCap   = m_cvThreshold * m_designCapacity;
        const double net     = (m_chargeToFull && m_currentCapacity >= cvCap) ? netFull * m_taperFraction : netFull;
        return -net;
    }

private:
    template<typename T>
    T clip(const T &n, const T &lower, const T &upper) {
        return std::max(lower, std::min(n, upper));
    }
};
