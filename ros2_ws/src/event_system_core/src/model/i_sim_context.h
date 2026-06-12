#pragma once

#include <memory>
#include <optional>
#include <random>
#include <string>

#include "robot_state.h"
#include "../util/types.h"
#include "../plugins/i_order.h"

class IEvent;
class Robot;
class Scheduler;

struct Journey {
    double duration;
    double distance;
};

// Interface for the simulation context used by events and BT nodes.
class ISimContext {
public:
    virtual ~ISimContext() = default;

    // Time
    virtual int getTime() const = 0;

    // Event queue
    virtual void pushEvent(const std::shared_ptr<IEvent>& event) = 0;

    // Push an event that marks the end of the robot's current activity
    // (StopDrive, BatteryFull, End*Event). Tracked as the in-flight commitment
    // so that interrupts can shift it. For everything else, use pushEvent.
    virtual void startActivity(const std::shared_ptr<IEvent>& endEvent) = 0;

    // Behaviour tree
    virtual void tickBT() = 0;
    virtual void setBTBlackboard(const std::string& key, const std::string& value) = 0;

    // Robot
    virtual std::shared_ptr<Robot> getRobot() const = 0;
    virtual void changeRobotState(std::unique_ptr<RobotState> newState) const = 0;
    virtual void robotMoved(const std::string& location, double distance = 0) const = 0;
    virtual Journey scheduleArrival(const std::string& target) const = 0;
    virtual const Scheduler& getScheduler() const = 0;

    // Path distance in meters between two waypoints, independent of the
    // distance source (matrix or Nav2). nullopt = unknown waypoint / no path.
    virtual std::optional<double> getDistance(const std::string& from, const std::string& to) const = 0;

    // Notifications
    virtual void notifyEvent(const IEvent& event) const = 0;
    virtual void notifyBatteryChanged() const = 0;

    // Mission management
    virtual void setOrderPtr(const des::OrderPtr& orderPtr) = 0;
    virtual des::OrderPtr getOrderPtr() const = 0;
    virtual void updateOrderState(const des::MissionState& newState) = 0;

    // scheduled
    virtual void addScheduledMission(const des::OrderPtr orderPtr) = 0;
    virtual bool hasScheduledMission() const = 0;
    virtual des::OrderPtr nextScheduledMission() = 0;
    virtual des::OrderPtr popScheduledMission() = 0;

    // Time of the next MissionDispatchEvent in the event queue, i.e. when the
    // next scheduled mission will become pending. nullopt = nothing scheduled.
    virtual std::optional<int> getNextScheduledDispatchTime() const = 0;

    // OrderPtr of the next scheduled mission still queued, or nullptr.
    virtual des::OrderPtr peekNextScheduledOrder() const = 0;


    // background
    virtual void addBackgroundMission(const des::OrderPtr orderPtr) = 0;
    virtual bool hasBackgroundMission() const = 0;
    virtual des::OrderPtr acceptFeasibleBackgroundMission() = 0;

    virtual std::optional<int> getSimulationEndTime() const = 0;
    virtual void completeOrder(const des::OrderPtr& appt) = 0;
    virtual void publishMission(const des::OrderPtr& order, int time) = 0;
    virtual void publishMissionRegistered(const des::OrderPtr& order) = 0;

    // Employee data
    virtual bool hasEmployee(const std::string& person) const = 0;
    virtual std::shared_ptr<des::Person> getPersonByName(const std::string& person) const = 0;

    // Person location registry (single source of truth)
    virtual std::string getPersonLocation(const std::string& name) const = 0;
    virtual void setPersonLocation(const std::string& name, const std::string& room) = 0;
    virtual double getLocationArea(const std::string& name) const = 0;

    // Configuration accessors
    virtual std::shared_ptr<des::SimConfig> getConfig() const = 0;

    // RNG access
    virtual std::mt19937& rng() const = 0;

    // Interrupts
    virtual bool pushInterrupt(const des::OrderPtr& order) = 0;
    virtual void popInterrupt(const des::OrderPtr& completedOrder) = 0;
    virtual bool hasActiveInterrupt() const = 0;
};
