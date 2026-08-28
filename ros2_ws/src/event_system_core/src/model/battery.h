/*
 * Energy model of the robot: capacity in Ah, updated from power draw
 * over elapsed time. Charging is two-phase CC/CV, tapered above
 * cvThreshold and stopping either at that knee or at the full
 * threshold, depending on the charge target.
 *
 */

#pragma once

#include <algorithm>

#include "../util/types.h"

namespace des {

class Battery {
    int m_lastBalanceUpdate = 0;

    double m_designCapacity;      // Ah - battery design capacity
    double m_currentCapacity;     // Ah
    double m_initialCapacity;     // Ah - on simulation start
    double m_lowBatteryThreshold; // %
    double m_fullBatteryThreshold; // %

    double m_dischargedAh  = 0.0;
    double m_voltage       = 12.0;
    double m_cvThreshold   = 0.8;
    double m_taperFraction = 0.5;
    bool m_chargeToFull    = true;
    bool m_forceFull       = false; // runtime override: top up to full even if m_chargeToFull is false
    bool m_depleted        = false;

    [[nodiscard]] bool chargesToFull() const;
    [[nodiscard]] double capacityToTime(double capacityDiff, double powerWatts) const;
    [[nodiscard]] double targetCapacity() const;

public:
    explicit Battery(
        double capacity,
        double initialCapacity,
        double lowBatteryThreshold,
        double fullBatteryThreshold,
        double voltage = 12.0,
        double cvThreshold = 0.8,
        double taperFraction = 0.5,
        bool chargeToFull = true
    );

    void updateConfig(
        double designCapacity,
        double initialCapacity,
        double lowThreshold,
        double fullThreshold,
        double voltage,
        double cvThreshold,
        double taperFraction,
        bool chargeToFull
        );

    // incrementally update battery capacity
    void updateBalance(int time, double energyConsumption);
    [[nodiscard]] double getDischargedAh() const;
    void completeCharge();

    // Runtime override of the charge target, independent of the configured charge_to_full baseline.
    void setForceFull(bool forceFull);
    [[nodiscard]] bool isDepleted() const;
    void reset(int startTime);
    [[nodiscard]] BatteryProps getStats() const;
    [[nodiscard]] double getVoltage() const;
    [[nodiscard]] bool isBatteryLow() const;
    [[nodiscard]] bool isFullyCharged() const;
    [[nodiscard]] double timeToFull(double phaseOnePowerWatts) const;
    [[nodiscard]] double timeToPhaseTransition(double phaseOnePowerWatts) const;
    [[nodiscard]] double chargingConsumption(double chargingRate, double baseConsumption) const;

private:
    template<typename T>
    T clip(const T &n, const T &lower, const T &upper) {
        return std::max(lower, std::min(n, upper));
    }
};

}  // namespace des
