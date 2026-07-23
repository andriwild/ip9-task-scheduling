#pragma once

#include <algorithm>

#include "../util/types.h"

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
    bool m_forceFull       = false; // runtime override: top up to full even if m_chargeToFull is false
    bool m_depleted        = false;

    bool chargesToFull() const;
    double capacityToTime(const double capacityDiff, const double powerWatts) const;
    double targetCapacity() const;

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
    );

    void updateConfig(const double designCapacity, const double initialCapacity, const double lowThreshold, const double fullThreshold,
                      const double voltage, const double cvThreshold, const double taperFraction, const bool chargeToFull);
    void updateBalance(const int time, const double energyConsumption);
    void completeCharge();

    // Runtime override of the charge target, independent of the configured charge_to_full baseline.
    void setForceFull(const bool forceFull);
    bool isDepleted() const;
    void reset(const int startTime);
    des::BatteryProps getStats() const;
    double getVoltage() const;
    bool isBatteryLow() const;
    bool isBatteryFull() const;
    bool isFullyCharged() const;
    double timeToFull(const double phaseOnePowerWatts) const;
    double timeToPhaseTransition(const double phaseOnePowerWatts) const;
    double chargingConsumption(const double chargingRate, const double baseConsumption) const;

private:
    template<typename T>
    T clip(const T &n, const T &lower, const T &upper) {
        return std::max(lower, std::min(n, upper));
    }
};
