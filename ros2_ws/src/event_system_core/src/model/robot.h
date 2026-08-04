#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "../util/log.h"
#include "robot_state.h"
#include "battery.h"
#include "sighting.h"

class IEvent;

class Robot {
    std::string m_dockLocation = "IMVS_Dock";
    std::unique_ptr<RobotState> m_state;
    std::string m_currentLocation;
    std::string m_targetLocation;
    des::Point m_position;
    des::Polygon m_visibility;

    SightingLog m_sightings;

    double m_driveSpeed;
    double m_currentSpeed = 0;

    // TODO: enum instead of single bools
    bool m_isDriving        = false;
    bool m_isCharging       = false;
    bool m_isScanning       = false;
    bool m_chargingRequired = false;
    bool m_isPersonVisible  = false;

    std::weak_ptr<IEvent> m_inFlightEvent;

    std::unique_ptr<Battery> m_bat;

public:
    bool m_batteryFullEventScheduled = false;
    bool m_opportunisticCharge = false;

    explicit Robot(const std::shared_ptr<des::SimConfig>& config);

    bool isBusy() const;
    void updateConfig(const des::SimConfig& config);

    std::string getLocation() const;
    void setLocation(const std::string& location);

    std::string getTargetLocation() const;
    void setTargetLocation(const std::string& location);

    des::Point getPosition() const;
    void setPosition(const des::Point& position);

    const des::Polygon& getVisibility() const;
    void setVisibility(const des::Polygon& visibility);

    void changeState(std::unique_ptr<RobotState> newState);
    RobotState* getState() const;

    bool isDriving() const;
    bool isPersonVisible() const;
    bool setIsPersonVisible(const bool isPersonVisible);

    void setDriving(const bool isDriving);

    bool isCharging() const;
    void setCharging(const bool isCharging);

    bool isScanning() const;
    void setScanning(const bool isScanning);

    bool updateAndGetChargingRequired();
    void setChargingRequired(const bool isChargingRequired);

    des::BatteryProps batteryStats() const;
    double batteryVoltage() const;
    bool isBatteryLow() const;
    bool isBatteryDepleted() const;
    bool isBatteryFullyCharged() const;
    double batteryTimeToFull(const double phaseOnePowerWatts) const;
    double batteryTimeToPhaseTransition(const double phaseOnePowerWatts) const;
    double chargingConsumption(const double chargingRate, const double baseConsumption) const;
    void updateBatteryBalance(const int time, const double energyConsumption);
    void completeCharge();
    void setBatteryForceFull(const bool forceFull);

    des::RobotStateType getStateType() const;

    double getCurrentSpeed() const;
    void setSpeed(const double newSpeed);

    double getDriveSpeed() const;
    void setDriveSpeed(const double speed);

    std::string getIdleLocation() const;

    std::weak_ptr<IEvent> inFlight() const;
    void setInFlight(const std::shared_ptr<IEvent>& event);
    void clearInFlight();

    void beginRoomVisit(const std::string& location);
    void observePerson(const int time, const std::string& personName, const bool seen);
    void flushRoomVisit();
    const SightingLog& getSightings() const;
};
