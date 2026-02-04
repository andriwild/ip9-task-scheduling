#include <iostream>

struct BatteryStats {
    double capacity;
    double soc;
};

class Battery {
    int m_lastBalanceUpdate = 0;

    double m_capacity; // Ah - battery design capacity
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
        : m_capacity(capacity)
        , m_initialCapacity(initialCapacity)
        , m_lowBatteryThreshold(lowBatteryThreshold)
    {
        m_currentCapacity = initialCapacity;
    }

    void updateBalance(int time, double energyConsumption) {
        // energy in Watt, time in seconds (+ discharge, - charge)
        // Ah = (W * s) / (3600 * V)
        int timeDelta = time - m_lastBalanceUpdate;
        m_lastBalanceUpdate = time;
        m_capacity -= energyConsumption * timeDelta / (3600 * m_voltage);
    }

    void reset(int startTime) {
        m_lastBalanceUpdate = startTime;
        m_currentCapacity = m_initialCapacity;
    };

    BatteryStats getStats() const { return {m_currentCapacity / m_capacity, m_capacity };};
    bool isBatteryLow() const { return m_capacity < m_lowBatteryThreshold; };
};
