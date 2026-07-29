#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cassert>
#include <map>
#include <utility>
#include <memory>
#include <optional>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include "mission/mission_board.h"
#include "person_registry.h"
#include "service_log.h"
#include "robot.h"
#include "robot_state.h"
#include "../observer/event_bus.h"
#include "../sim/i_path_planner.h"
#include "../model/i_sim_context.h"
#include "../util/log.h"
#include "../util/types.h"
#include "../model/event_queue.h"


class SimulationContext : public ISimContext {
    int m_currentTime {};
    std::shared_ptr<des::SimConfig> m_simConfig;
    EventBus m_eventBus;
    EventQueue& m_queue;
    std::shared_ptr<BT::Tree> m_behaviorTree;
    const IEvent* m_currentEvent = nullptr;

    std::unique_ptr<Scheduler> m_scheduler;

    static constexpr unsigned int DEFAULT_SEED = 42;
    // mutable: RNG state changes are an implementation detail, allowing use in const methods
    mutable std::mt19937 m_worldRng{DEFAULT_SEED};
    mutable std::mt19937 m_robotRng{DEFAULT_SEED + des::ROBOT_SEED_OFFSET};
    unsigned int m_activeSeed = DEFAULT_SEED;

    std::shared_ptr<IPathPlanner> m_plannerNode;
    std::unique_ptr<Robot> m_robot;
    des::LocationMap m_locationMap;
    des::RoomTourMap m_roomTours;
    PersonRegistry m_persons;
    ServiceLog m_services;
    MissionBoard m_missions;

public:
    explicit SimulationContext(
        EventQueue& queue,
        std::shared_ptr<des::SimConfig> simConfig,
        std::shared_ptr<IPathPlanner> plannerNode,
        des::PersonMap employees,
        des::LocationMap locationMap,
        des::RoomTourMap roomTours
    );

    Scheduler& getScheduler();
    const Scheduler& getScheduler() const override;

    void resetRobot();

    Journey scheduleArrival(const std::string& target) const override;
    std::optional<double> getDistance(const std::string& from, const std::string& to) const override;

    void changeRobotState(std::unique_ptr<RobotState> newState) const override;
    void setConfig(const std::shared_ptr<des::SimConfig>& newConfig);
    void resetContext(int newTime);
    void completeOrder(const des::OrderPtr& order) override;

    int getTime() const override;
    std::shared_ptr<des::SimConfig> getConfig() const override;
    Robot* getRobot() const override;
    std::mt19937& worldRng() const override;
    std::mt19937& robotRng() const override;
    void reseed(unsigned int seed);
    void reseedPersons();
    unsigned int activeSeed() const;

    void pushEvent(const std::shared_ptr<IEvent>& event) override;
    void startActivity(const std::shared_ptr<IEvent>& endEvent) override;
    void executeEvent(const std::shared_ptr<IEvent>& event);
    void printEventQueue() const;

    void setBehaviorTree(std::shared_ptr<BT::Tree> tree);
    void tickBT() override;
    void setBTBlackboard(const std::string& key, const std::string& value) override;

    des::Person* getPersonByName(const std::string& person) const override;
    bool hasEmployee(const std::string& person) const override;
    std::string getPersonLocation(const std::string& name) const override;
    const std::map<std::string, std::string>& getAllPersonLocations() const override;
    void setPersonLocation(const std::string& name, const std::string& room) override;
    std::optional<des::Point> getPersonPosition(const std::string& name) const override;
    bool robotSeesPerson(const std::string& name) const override;

    std::optional<int> lastServiced(const std::string& room, const std::string& type) const override;
    void recordServiced(const std::string& room, const std::string& type, int time) override;

    std::vector<std::string> roomNames() const override;
    const des::Location& location(const std::string& room) const override;
    const des::RoomTour* roomTour(const std::string& room) const override;

    // Mission management — current mission plus the three mission channels.
    void setOrderPtr(const des::OrderPtr& orderPtr) override;
    des::OrderPtr getOrderPtr() const override;
    void updateOrderState(const des::MissionState& newState) override;

    void addScheduledMission(const des::OrderPtr orderPtr) override;
    bool hasScheduledMission() const override;
    des::OrderPtr nextScheduledMission() override;
    des::OrderPtr popScheduledMission() override;

    void addBackgroundMission(const des::OrderPtr orderPtr) override;
    bool hasBackgroundMission() const override;
    des::OrderPtr acceptFeasibleBackgroundMission() override;

    std::optional<int> getNextScheduledDispatchTime() const override;
    des::OrderPtr peekNextScheduledOrder() const override;
    std::vector<des::OrderPtr> peekScheduledOrdersUntil(int untilTime) const override;
    std::optional<int> getSimulationEndTime() const override;

    void publishMission(const des::OrderPtr& order, int time) override;
    void publishMissionRegistered(const des::OrderPtr& order) override;

    void advanceTime(int newTime);

    // Observer functions (delegated to EventBus)
    void addObserver(const std::shared_ptr<IObserver>& observer);
    void removeObserver(const std::shared_ptr<IObserver>& observer);
    void clearObservers();

    void notifyMissionComplete(const des::OrderPtr& order, int timeDiff) const;
    void notifyRobotStateChanged() const;
    void notifyBatteryChanged() const override;
    void notifyEvent(const IEvent& event) const override;
    void notifyChargeStarted() const override;
    void robotMoved(const std::string& location, double distance = 0) const override;
    void robotMovedTo(const des::Point& position, double distance = 0.0) const override;

    double getDriveTimeStd() const;

    bool pushInterrupt(const des::OrderPtr& order) override;
    void popInterrupt(const des::OrderPtr& completedOrder) override;
    bool hasActiveInterrupt() const override;
};
