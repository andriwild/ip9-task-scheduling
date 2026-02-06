#include "../util/types.h"

class Battery {
    int m_lastBalanceUpdate = 0;

    double m_designCapactiy; // Ah - battery design capacity
    double m_currentCapacity; // Ah
    double m_initialCapacity; // Ah - on simulation start
    double m_lowBatteryThreshold;
    double m_voltage = 12.0; // Volt

public:
    explicit Battery(
        const double capacity,
        const double initialCapacity,
        const double lowBatteryThreshold
    )
        : m_designCapactiy(capacity)
        , m_initialCapacity(initialCapacity)
        , m_lowBatteryThreshold(lowBatteryThreshold)
    {
        m_currentCapacity = initialCapacity;
    }

    void updateConfig(double designCapacity, double initialCapacity, double threshold) {
        m_designCapactiy      = designCapacity; 
        m_initialCapacity     = initialCapacity;
        m_lowBatteryThreshold = threshold;
    }

    void updateBalance(int time, double energyConsumption) {
        // energy in Watt, time in seconds (+ discharge, - charge)
        // Ah = (W * s) / (3600 * V)
        int timeDelta = time - m_lastBalanceUpdate;
        m_lastBalanceUpdate = time;
        m_currentCapacity -= energyConsumption * timeDelta / (3600 * m_voltage);
    }

    void reset(int startTime) {
        m_lastBalanceUpdate = startTime;
        m_currentCapacity = m_initialCapacity;
    }

    des::BatteryProps getStats() const { 
        return { m_currentCapacity / m_designCapactiy, m_designCapactiy, m_lowBatteryThreshold };
    }

    bool isBatteryLow() const { return m_designCapactiy < m_lowBatteryThreshold; };
};
