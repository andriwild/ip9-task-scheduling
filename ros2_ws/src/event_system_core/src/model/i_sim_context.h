#pragma once

#include <memory>
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
// Separates the simulation API from setup/lifecycle concerns (reset, config, observers).
class ISimContext {
public:
    virtual ~ISimContext() = default;

    // Time
    virtual int getTime() const = 0;

    // Event queue
    virtual void pushEvent(const std::shared_ptr<IEvent>& event) = 0;

    // Behaviour tree
    virtual void tickBT() = 0;
    virtual void setBTBlackboard(const std::string& key, const std::string& value) = 0;

    // Robot
    virtual std::shared_ptr<Robot> getRobot() const = 0;
    virtual void changeRobotState(std::unique_ptr<RobotState> newState) const = 0;
    virtual void robotMoved(const std::string& location, double distance = 0) const = 0;
    virtual Journey scheduleArrival(const std::string& target) const = 0;
    virtual const Scheduler& getScheduler() const = 0;

    // Notifications
    virtual void notifyEvent(const IEvent& event) const = 0;

    // Mission management
    virtual void setOrderPtr(const des::OrderPtr& orderPtr) = 0;
    virtual des::OrderPtr getOrderPtr() const = 0;
    virtual void updateOrderState(const des::MissionState& newState) = 0;
    virtual void addPendingMission(const des::OrderPtr orderPtr) = 0;
    virtual bool hasPendingMission() const = 0;
    virtual des::OrderPtr nextPendingMission() = 0;
    virtual des::OrderPtr popPendingMission() = 0;
    virtual void completeOrder(const des::OrderPtr& appt) = 0;

    // Employee data
    virtual bool hasEmployee(const std::string& person) const = 0;
    virtual std::shared_ptr<des::Person> getPersonByName(const std::string& person) const = 0;

    // Person location registry (single source of truth)
    virtual std::string getPersonLocation(const std::string& name) const = 0;
    virtual void setPersonLocation(const std::string& name, const std::string& room) = 0;
    virtual double getSearchArea(const std::string& name) const = 0;

    // Configuration accessors
    virtual std::shared_ptr<des::SimConfig> getConfig() const = 0;
    virtual double getConversationProbability() const = 0;
    virtual double getRndConversationTime() const = 0;

    // RNG access
    virtual std::mt19937& rng() const = 0;
};
