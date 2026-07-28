#pragma once

#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

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

class ISimClock {
public:
    virtual ~ISimClock() = default;
    virtual int getTime() const = 0;
    virtual std::optional<int> getSimulationEndTime() const = 0;
};

class IEventSink {
public:
    virtual ~IEventSink() = default;
    virtual void pushEvent(const std::shared_ptr<IEvent>& event) = 0;

    // Push an event that marks the end of the robot's current activity
    // (StopDrive, BatteryFull, End*Event). Tracked as the in-flight commitment
    // so that interrupts can shift it. For everything else, use pushEvent.
    virtual void startActivity(const std::shared_ptr<IEvent>& endEvent) = 0;
};

class IBehaviorTreeAccess {
public:
    virtual ~IBehaviorTreeAccess() = default;
    virtual void tickBT() = 0;
    virtual void setBTBlackboard(const std::string& key, const std::string& value) = 0;
};

class IRobotAccess {
public:
    virtual ~IRobotAccess() = default;
    virtual Robot* getRobot() const = 0;
    virtual void changeRobotState(std::unique_ptr<RobotState> newState) const = 0;
    virtual void robotMoved(const std::string& location, double distance = 0) const = 0;
    virtual void robotMovedTo(const des::Point& position, double distance = 0.0) const = 0;
};

class IPathPlanning {
public:
    virtual ~IPathPlanning() = default;
    virtual Journey scheduleArrival(const std::string& target) const = 0;
    virtual const Scheduler& getScheduler() const = 0;

    // Path distance in meters between two waypoints, independent of the
    // distance source (matrix or Nav2). nullopt = unknown waypoint / no path.
    virtual std::optional<double> getDistance(const std::string& from, const std::string& to) const = 0;
};

class INotifier {
public:
    virtual ~INotifier() = default;
    virtual void notifyEvent(const IEvent& event) const = 0;
    virtual void notifyBatteryChanged() const = 0;
    virtual void notifyChargeStarted() const = 0;
    virtual void publishMission(const des::OrderPtr& order, int time) = 0;
    virtual void publishMissionRegistered(const des::OrderPtr& order) = 0;
};

class IMissionBoard {
public:
    virtual ~IMissionBoard() = default;

    virtual void setOrderPtr(const des::OrderPtr& orderPtr) = 0;
    virtual des::OrderPtr getOrderPtr() const = 0;
    virtual void updateOrderState(const des::MissionState& newState) = 0;
    virtual void completeOrder(const des::OrderPtr& appt) = 0;

    virtual void addScheduledMission(const des::OrderPtr orderPtr) = 0;
    virtual bool hasScheduledMission() const = 0;
    virtual des::OrderPtr nextScheduledMission() = 0;
    virtual des::OrderPtr popScheduledMission() = 0;

    // Time of the next MissionDispatchEvent in the event queue, i.e. when the
    // next scheduled mission will become pending. nullopt = nothing scheduled.
    virtual std::optional<int> getNextScheduledDispatchTime() const = 0;

    // OrderPtr of the next scheduled mission still queued, or nullptr.
    virtual des::OrderPtr peekNextScheduledOrder() const = 0;

    virtual std::vector<des::OrderPtr> peekScheduledOrdersUntil(int untilTime) const = 0;

    virtual void addBackgroundMission(const des::OrderPtr orderPtr) = 0;
    virtual bool hasBackgroundMission() const = 0;
    virtual des::OrderPtr acceptFeasibleBackgroundMission() = 0;

    virtual bool pushInterrupt(const des::OrderPtr& order) = 0;
    virtual void popInterrupt(const des::OrderPtr& completedOrder) = 0;
    virtual bool hasActiveInterrupt() const = 0;
};

class IPersonRegistry {
public:
    virtual ~IPersonRegistry() = default;
    virtual bool hasEmployee(const std::string& person) const = 0;
    virtual des::Person* getPersonByName(const std::string& person) const = 0;

    // Person location registry (single source of truth)
    virtual std::string getPersonLocation(const std::string& name) const = 0;
    virtual const std::map<std::string, std::string>& getAllPersonLocations() const = 0;
    virtual void setPersonLocation(const std::string& name, const std::string& room) = 0;
    virtual std::optional<des::Point> getPersonPosition(const std::string& name) const = 0;
    virtual bool robotSeesPerson(const std::string& name) const = 0;
};

class IWorldModel {
public:
    virtual ~IWorldModel() = default;
    virtual const des::Location& location(const std::string& room) const = 0;
    virtual const des::RoomTour* roomTour(const std::string& room) const = 0;
    virtual std::vector<std::string> roomNames() const = 0;
    virtual std::optional<int> lastServiced(const std::string& room, const std::string& type) const = 0;
    virtual void recordServiced(const std::string& room, const std::string& type, int time) = 0;
};

class IConfigAccess {
public:
    virtual ~IConfigAccess() = default;
    virtual std::shared_ptr<des::SimConfig> getConfig() const = 0;
};

class IRngAccess {
public:
    virtual ~IRngAccess() = default;
    virtual std::mt19937& rng() const = 0;
};

class ISimContext
    : public ISimClock
    , public IEventSink
    , public IBehaviorTreeAccess
    , public IRobotAccess
    , public IPathPlanning
    , public INotifier
    , public IMissionBoard
    , public IPersonRegistry
    , public IWorldModel
    , public IConfigAccess
    , public IRngAccess {
public:
    ~ISimContext() override = default;
};
