#include "battery.h"


#include "../util/log.h"

namespace des {

bool Battery::chargesToFull() const {
    return m_chargeToFull || m_forceFull;
}

double Battery::capacityToTime(const double capacityDiff, const double powerWatts) const {
    return capacityDiff * 3600.0 * m_voltage / powerWatts;
}

double Battery::targetCapacity() const {
    const double cvCap   = m_cvThreshold * m_designCapacity;
    const double fullCap = m_fullBatteryThreshold / 100.0 * m_designCapacity;
    return chargesToFull() ? fullCap : cvCap;
}

Battery::Battery(
    const double capacity,
    const double initialCapacity,
    const double lowBatteryThreshold,
    const double fullBatteryThreshold,
    const double voltage,
    const double cvThreshold,
    const double taperFraction,
    const bool chargeToFull
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

void Battery::updateConfig(
    const double designCapacity,
    const double initialCapacity,
    const double lowThreshold,
    const double fullThreshold,
    const double voltage,
    const double cvThreshold,
    const double taperFraction,
    const bool chargeToFull
) {
    m_designCapacity       = designCapacity;
    m_initialCapacity      = initialCapacity;
    m_lowBatteryThreshold  = lowThreshold;
    m_fullBatteryThreshold = fullThreshold;
    m_voltage              = voltage;
    m_cvThreshold          = cvThreshold;
    m_taperFraction        = taperFraction;
    m_chargeToFull         = chargeToFull;
    DES_LOG_DEBUG("des.battery", "Config updated");
}

void Battery::updateBalance(const int time, const double energyConsumption) {
    // energy in Watt, time in seconds (+ discharge, - charge)
    // Ah = (W * s) / (3600 * V)
    const int timeDelta = time - m_lastBalanceUpdate;
    m_lastBalanceUpdate = time;
    const double capacityDiff = energyConsumption * timeDelta / (3600 * m_voltage);
    const double capacityBefore = m_currentCapacity;
    m_currentCapacity -= capacityDiff;


    if (m_currentCapacity < m_lowBatteryThreshold / 100 * m_designCapacity) {
        DES_LOG_DEBUG("des.battery", "Battery Low - SOC: %.1f", m_currentCapacity / m_designCapacity);
    }

    if (m_currentCapacity <= 0) {
        DES_LOG_ERROR("des.battery", "Battery discharged - no energy left");
        m_depleted = true;
    }
    m_currentCapacity = clip(m_currentCapacity, 0.0, m_designCapacity);
    if (capacityBefore > m_currentCapacity) {
        m_dischargedAh += capacityBefore - m_currentCapacity;
    }
}

double Battery::getDischargedAh() const {
    return m_dischargedAh;
}

void Battery::completeCharge() {
    m_currentCapacity = targetCapacity();
}

void Battery::setForceFull(const bool forceFull) {
    m_forceFull = forceFull;
}

bool Battery::isDepleted() const {
    return m_depleted;
}

void Battery::reset(const int startTime) {
    m_lastBalanceUpdate = startTime;
    m_currentCapacity = m_initialCapacity;
    m_forceFull = false;
    m_depleted = false;
    m_dischargedAh = 0.0;
    DES_LOG_DEBUG("des.battery", "Reset: initial capactiy: %.1f", m_initialCapacity);
}

BatteryProps Battery::getStats() const {
    return { m_currentCapacity / m_designCapacity, m_designCapacity, m_lowBatteryThreshold };
}

double Battery::getVoltage() const {
    return m_voltage;
}

bool Battery::isBatteryLow() const {
    const bool isLow = m_currentCapacity < m_lowBatteryThreshold / 100 * m_designCapacity;
    return isLow;
}

bool Battery::isFullyCharged() const {
    return m_currentCapacity >= targetCapacity() - 1e-6;
}

double Battery::timeToFull(const double phaseOnePowerWatts) const {
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
    DES_LOG_DEBUG("des.battery", "Calculate time to full: %f", duration);
    return duration;
}

double Battery::timeToPhaseTransition(const double phaseOnePowerWatts) const {
    if (phaseOnePowerWatts <= 0) return -1.0;
    if (!chargesToFull()) return -1.0;

    const double cvCap   = m_cvThreshold * m_designCapacity;
    const double fullCap = m_fullBatteryThreshold / 100.0 * m_designCapacity;
    if (fullCap <= cvCap) return -1.0;
    if (m_currentCapacity >= cvCap) return -1.0;

    return capacityToTime(cvCap - m_currentCapacity, phaseOnePowerWatts);
}

double Battery::chargingConsumption(const double chargingRate, const double baseConsumption) const {
    const double netFull = chargingRate - baseConsumption;
    const double cvCap   = m_cvThreshold * m_designCapacity;
    const double net     = (chargesToFull() && m_currentCapacity >= cvCap) ? netFull * m_taperFraction : netFull;
    return -net;
}

}  // namespace des
