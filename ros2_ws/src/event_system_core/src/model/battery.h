#include <iostream>

class Battery {
    double m_energy = 100.0;
    double m_capacity;
    double m_chargingRate;
    double m_lowBatteryThreshold;

public:
    explicit Battery(
        const double batteryCapacity,
        const double chargingRate,
        const double lowBatteryThreshold
    ):
        m_capacity(batteryCapacity),
        m_chargingRate(chargingRate),
        m_lowBatteryThreshold(lowBatteryThreshold)
    {}

    void updateBalance(int time, double energyConsumption) {
        int timeDelta = time - lastTime;
        std::cout << "[BAT] Balance update for: " << timeDelta << std::endl;
        lastTime = time;
    }

    void reset() {
        std::cout << "[BAT] Reset"<< std::endl;
        m_energy = 100.0;
    };

    double getCapacity() const { return m_capacity; };

    bool isBatteryLow() const { return m_energy < m_lowBatteryThreshold; };

private:
    int lastTime = 0;
};
