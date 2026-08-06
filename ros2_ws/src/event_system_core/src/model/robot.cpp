#include "robot.h"
#include "../util/log.h"
#include "robot_state.h"
#include "engine/contracts/i_event.h"

Robot::Robot(const std::shared_ptr<des::SimConfig>& config, const int startTime)
    : m_state(std::make_unique<IdleState>())
{
    m_now = startTime;
    m_driveSpeed   = config->robotSpeed;
    m_dockLocation = config->dockLocation;

    m_bat = std::make_unique<Battery>(
        config->batteryCapacity,
        config->initialBatteryCapacity,
        config->lowBatteryThreshold,
        config->fullBatteryThreshold,
        config->batteryVoltage,
        config->cvThreshold,
        config->taperFraction,
        config->chargeToFull
    );
    setLocation(m_dockLocation);

    // enter initial state to make sure protocol logging is active
    m_state->enter(*this);
}

void Robot::changeState(std::unique_ptr<RobotState> newState, const int time) {
    m_now = time;
    if (m_state) {
        m_state->exit(*this);
    }
    m_state = std::move(newState);
    m_state->enter(*this);
}

void Robot::closeStateLog(const int time) {
    m_now = time;
    m_stateLog.close(time, m_bat->getStats().soc);
}

void Robot::beginStateInterval(const des::RobotStateType category, std::string name) {
    m_stateLog.open(m_now, category, std::move(name), m_bat->getStats().soc);
}

void Robot::endStateInterval() {
    m_stateLog.close(m_now, m_bat->getStats().soc);
}

const StateLog& Robot::getStateLog() const {
    return m_stateLog;
}

void Robot::addDistance(const double distance) {
    m_odometer += distance;
}

double Robot::getOdometer() const {
    return m_odometer;
}

void Robot::beginChargeSession(const int time) {
    m_chargeSessions.push_back(time);
}

const std::vector<int>& Robot::getChargeSessions() const {
    return m_chargeSessions;
}

double Robot::getDischargedAh() const {
    return m_bat->getDischargedAh();
}

bool Robot::isBusy() const {
    const auto type = m_state->getType();
    return type != des::RobotStateType::IDLE || isDriving();
}

void Robot::updateConfig(const des::SimConfig& config) {
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot"), "Robot: Updating configuration");
    setDriveSpeed(config.robotSpeed);
    m_dockLocation = config.dockLocation;
    m_bat->updateConfig(
        config.batteryCapacity,
        config.initialBatteryCapacity,
        config.lowBatteryThreshold,
        config.fullBatteryThreshold,
        config.batteryVoltage,
        config.cvThreshold,
        config.taperFraction,
        config.chargeToFull
    );
}

std::string Robot::getLocation() const {
    return m_currentLocation;
}

void Robot::setLocation(const std::string& location) {
    m_currentLocation = location;
    if (m_currentLocation == getIdleLocation()) {
        m_isCharging = true;
    }
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot"), "Robot location set to: %s", location.c_str());
}

des::Point Robot::getPosition() const {
    return m_position;
}

void Robot::setPosition(const des::Point& position) {
    m_position = position;
}

const des::Polygon& Robot::getVisibility() const {
    return m_visibility;
}

void Robot::setVisibility(const des::Polygon& visibility) {
    m_visibility = visibility;
}

std::string Robot::getTargetLocation() const {
    return m_targetLocation;
}

void Robot::setTargetLocation(const std::string& location) {
    m_targetLocation = location;
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot"), "Robot target location set to: %s", location.c_str());
}

RobotState* Robot::getState() const {
    return m_state.get();
}

bool Robot::isDriving() const {
    return m_isDriving;
}

bool Robot::isPersonVisible() const {
    return m_isPersonVisible;
}

bool Robot::setIsPersonVisible(const bool isPersonVisible) {
    return m_isPersonVisible = isPersonVisible;
}

void Robot::setDriving(const bool isDriving) {
    m_isDriving = isDriving;
}

bool Robot::isCharging() const {
    return m_isCharging;
}

void Robot::setCharging(const bool isCharging) {
    m_isCharging = isCharging;
}

bool Robot::updateAndGetChargingRequired() {
    m_chargingRequired = false;
    if (m_bat->isBatteryLow()) {
        m_chargingRequired = true;
    }
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot"), "Robot charging required: %d", m_chargingRequired);
    return m_chargingRequired;
}

void Robot::setChargingRequired(const bool isChargingRequired) {
    m_chargingRequired = isChargingRequired;
}

des::BatteryProps Robot::batteryStats() const {
    return m_bat->getStats();
}

double Robot::batteryVoltage() const {
    return m_bat->getVoltage();
}

bool Robot::isBatteryLow() const {
    return m_bat->isBatteryLow();
}

bool Robot::isBatteryDepleted() const {
    return m_bat->isDepleted();
}

bool Robot::isBatteryFullyCharged() const {
    return m_bat->isFullyCharged();
}

double Robot::batteryTimeToFull(const double phaseOnePowerWatts) const {
    return m_bat->timeToFull(phaseOnePowerWatts);
}

double Robot::batteryTimeToPhaseTransition(const double phaseOnePowerWatts) const {
    return m_bat->timeToPhaseTransition(phaseOnePowerWatts);
}

double Robot::chargingConsumption(const double chargingRate, const double baseConsumption) const {
    return m_bat->chargingConsumption(chargingRate, baseConsumption);
}

void Robot::updateBatteryBalance(const int time, const double energyConsumption) {
    m_bat->updateBalance(time, energyConsumption);
}

void Robot::completeCharge() {
    m_bat->completeCharge();
}

void Robot::setBatteryForceFull(const bool forceFull) {
    m_bat->setForceFull(forceFull);
}

des::RobotStateType Robot::getStateType() const {
    return m_state->getType();
}

double Robot::getCurrentSpeed() const {
    return m_currentSpeed;
}

void Robot::setSpeed(const double newSpeed) {
    m_currentSpeed = newSpeed;
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot"), "Robot speed set to: %.2f", newSpeed);
}

double Robot::getDriveSpeed() const {
    return m_driveSpeed;
}

void Robot::setDriveSpeed(const double speed) {
    m_driveSpeed = speed;
    DES_LOG_DEBUG(rclcpp::get_logger("des.robot"), "Robot drive speed set to: %.2f", speed);
}

std::string Robot::getIdleLocation() const {
    return m_dockLocation;
}

std::weak_ptr<IEvent> Robot::inFlight() const {
    return m_inFlightEvent;
}

void Robot::setInFlight(const std::shared_ptr<IEvent>& event) {
    m_inFlightEvent = event;
}

void Robot::clearInFlight() {
    m_inFlightEvent.reset();
}

void Robot::beginRoomVisit(const std::string& location) {
    m_sightings.beginVisit(location);
}

void Robot::observePerson(const int time, const std::string& personName, const bool seen) {
    m_sightings.observe(time, personName, seen);
}

void Robot::flushRoomVisit() {
    m_sightings.flushVisit();
}

const SightingLog& Robot::getSightings() const {
    return m_sightings;
}
