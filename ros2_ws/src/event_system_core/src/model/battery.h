class Battery {
    double m_energy = 100.0;
    double m_energyConsumptionDrive;
    double m_energyConsumptionBase;
    double m_batteryCapacity;
    double m_chargingRate;
    double m_lowBatteryThreshold;

public:
    explicit Battery(
        const double energyConsumptionBase,
        const double energyConsumptionDrive,
        const double batteryCapacity,
        const double chargingRate,
        const double lowBatteryThreshold
    ):
        m_energyConsumptionDrive(energyConsumptionDrive),
        m_energyConsumptionBase(energyConsumptionBase),
        m_batteryCapacity(batteryCapacity),
        m_chargingRate(chargingRate),
        m_lowBatteryThreshold(lowBatteryThreshold)
    {}

    void discharge(double speed, double distance) {}

    void discharge(int time) {}

    void charge(int time) {}
};
