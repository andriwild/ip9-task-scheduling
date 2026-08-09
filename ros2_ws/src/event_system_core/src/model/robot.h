/*
 * State of the robot: location, position, visibility polygon speed and the current RobotState.
 * Owner of the battery and the sighting log it feeds while driving.
 *
 */
#pragma once

#include <memory>
#include <vector>

#include "../util/log.h"
#include "robot_state.h"
#include "battery.h"
#include "sighting.h"
#include "state_log.h"

namespace des {

class IEvent;

class Robot {
    std::string m_dockLocation;
    std::unique_ptr<RobotState> m_state;
    std::string m_currentLocation;
    std::string m_targetLocation;
    Point m_position;
    Polygon m_visibility;

    SightingLog m_sightings;
    StateLog m_stateLog;
    std::vector<int> m_chargeSessions;
    int m_now = 0;
    double m_odometer = 0.0;

    double m_driveSpeed;
    double m_currentSpeed = 0;

    // TODO: enum instead of single bools
    bool m_isDriving        = false;
    bool m_isCharging       = false;
    bool m_chargingRequired = false;
    bool m_isPersonVisible  = false;

    std::weak_ptr<IEvent> m_inFlightEvent;

    std::unique_ptr<Battery> m_bat;

public:
    bool m_batteryFullEventScheduled = false;
    bool m_opportunisticCharge = false;

    explicit Robot(const std::shared_ptr<SimConfig>& config, int startTime = 0);

    bool isBusy() const;
    void updateConfig(const SimConfig& config);

    std::string getLocation() const;
    void setLocation(const std::string& location);

    std::string getTargetLocation() const;
    void setTargetLocation(const std::string& location);

    Point getPosition() const;
    void setPosition(const Point& position);

    const Polygon& getVisibility() const;
    void setVisibility(const Polygon& visibility);

    void changeState(std::unique_ptr<RobotState> newState, int time);
    RobotState* getState() const;

    void closeStateLog(int time);
    void beginStateInterval(RobotStateType category, std::string name);
    void endStateInterval();
    const StateLog& getStateLog() const;

    void addDistance(double distance);
    double getOdometer() const;

    void beginChargeSession(int time);
    const std::vector<int>& getChargeSessions() const;
    double getDischargedAh() const;

    bool isDriving() const;
    bool isPersonVisible() const;
    bool setIsPersonVisible(const bool isPersonVisible);

    void setDriving(const bool isDriving);

    bool isCharging() const;
    void setCharging(const bool isCharging);

    bool updateAndGetChargingRequired();
    void setChargingRequired(const bool isChargingRequired);

    BatteryProps batteryStats() const;
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

    RobotStateType getStateType() const;

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

}  // namespace des
