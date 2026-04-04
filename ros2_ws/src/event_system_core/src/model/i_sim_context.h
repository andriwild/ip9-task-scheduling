#pragma once

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "../util/types.h"
#include "robot_state.h"

class IEvent;
class Robot;

struct Journey;

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

    // Notifications
    virtual void notifyEvent(const IEvent& event) const = 0;

    // Mission management
    virtual void setAppointment(const std::shared_ptr<des::Appointment>& appointment) = 0;
    virtual std::shared_ptr<des::Appointment> getAppointment() const = 0;
    virtual void updateAppointmentState(const des::MissionState& newState) = 0;
    virtual void addPendingMission(const std::shared_ptr<des::Appointment>& appointment) = 0;
    virtual bool hasPendingMission() const = 0;
    virtual std::shared_ptr<des::Appointment> nextPendingMission() = 0;
    virtual std::shared_ptr<des::Appointment> popPendingMission() = 0;
    virtual void completeAppointment(const std::shared_ptr<des::Appointment>& appt) const = 0;

    // Mission feasibility
    virtual bool isMissionFeasible(const des::Appointment& appointment, const std::string& startPos) const = 0;

    // Employee data
    virtual bool hasEmployee(const std::string& person) const = 0;
    virtual std::shared_ptr<des::Person> getPersonByName(const std::string& person) const = 0;

    // Configuration accessors
    virtual std::shared_ptr<des::SimConfig> getConfig() const = 0;
    virtual double getConversationProbability() const = 0;
    virtual double getRndConversationTime() const = 0;

    // RNG access
    virtual std::mt19937& rng() const = 0;
};
