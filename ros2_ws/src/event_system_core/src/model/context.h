#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cassert>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include "mission_manager.h"
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
    MissionManager m_missionManager;
    EventQueue& m_queue;
    std::shared_ptr<BT::Tree> m_behaviorTree;
    std::map<std::string, std::shared_ptr<des::Person>> m_employeeLocations;
    std::map<std::string, std::string> m_personLocations;
    std::unique_ptr<Scheduler> m_scheduler;

    struct InterruptSnapshot {
        std::unique_ptr<RobotState> state;
        bool wasDriving = false;
    };
    std::optional<InterruptSnapshot> m_interruptSnapshot;

public:
    static constexpr unsigned int DEFAULT_SEED = 42;
    // mutable: RNG state changes are an implementation detail, allowing use in const methods
    mutable std::mt19937 m_rng{DEFAULT_SEED};

    std::shared_ptr<IPathPlanner> m_plannerNode;
    std::shared_ptr<Robot> m_robot;
    des::LocationMap m_locationMap;

    explicit SimulationContext(
        EventQueue& queue,
        std::shared_ptr<des::SimConfig> simConfig,
        std::shared_ptr<IPathPlanner> plannerNode,
        std::map<std::string, std::shared_ptr<des::Person>> employeeLocations,
        des::LocationMap locationMap
    );

    Scheduler& getScheduler() { return *m_scheduler; }

    const Scheduler& getScheduler() const { return *m_scheduler; }

    void resetRobot() {
        m_robot.reset();
        m_robot = std::make_unique<Robot>(m_simConfig);
    }

    Journey scheduleArrival(const std::string& target) const override;
    void changeRobotState(std::unique_ptr<RobotState> newState) const override;
    void setConfig(const std::shared_ptr<des::SimConfig> &newConfig);
    void resetContext(int newTime);
    void completeOrder(const des::OrderPtr& order) override;

    int getTime() const override { return m_currentTime; }

    std::shared_ptr<des::SimConfig> getConfig() const override { return m_simConfig; }

    std::shared_ptr<Robot> getRobot() const override { return m_robot; }

    std::mt19937& rng() const override { return m_rng; }

    // Event queue access
    void pushEvent(const std::shared_ptr<IEvent>& event) override {
        m_queue.push(event);
    }

    void startActivity(const std::shared_ptr<IEvent>& endEvent) override {
        m_robot->setInFlight(endEvent);
        m_queue.push(endEvent);
    }

    void executeEvent(const std::shared_ptr<IEvent>& event) {
        event->execute(*this);
        if (m_robot->inFlight().lock() == event) {
            m_robot->clearInFlight();
        }
    }


    void printEventQueue() const { m_queue.print(); }

    // Behaviour tree access
    void setBehaviorTree(std::shared_ptr<BT::Tree> tree) {
        m_behaviorTree = std::move(tree);
    }

    void tickBT() override {
        m_behaviorTree->tickOnce();
    }

    void setBTBlackboard(const std::string& key, const std::string& value) override {
        m_behaviorTree->rootBlackboard()->set(key, value);
    }

    // Employee data access
    void updateEmployeeLocations(const std::map<std::string, std::shared_ptr<des::Person>>& locations) {
        m_employeeLocations = locations;
    }

    std::shared_ptr<des::Person> getPersonByName(const std::string& person) const override {
        return m_employeeLocations.at(person);
    }

    bool hasEmployee(const std::string& person) const override {
        return m_employeeLocations.contains(person);
    }

    std::string getPersonLocation(const std::string& name) const override {
        return m_personLocations.at(name);
    }

    void setPersonLocation(const std::string& name, const std::string& room) override {
        m_personLocations[name] = room;
    }

    double getLocationArea(const std::string& name) const override {
        auto it = m_locationMap.find(name);
        if (it == m_locationMap.end() || !it->second.m_area.has_value()) {
            DES_LOG_WARN(rclcpp::get_logger("des.context"), "Location area not found for '%s', defaulting to 1.0", name.c_str());
            return 1.0;
        }
        return it->second.m_area.value();
    }

    // Mission management (delegated to MissionManager)
    void setOrderPtr(const des::OrderPtr& orderPtr) override {
        m_missionManager.setCurrent(orderPtr);
    }

    des::OrderPtr getOrderPtr() const override {
        if (auto interrupt = m_missionManager.activeInterrupt()) {
            return interrupt;
        }
        return m_missionManager.getCurrent();
    }

    void updateOrderState(const des::MissionState& newState) override {
        m_missionManager.updateState(newState);
    }

    void addPendingMission(const des::OrderPtr orderPtr) override {
        m_missionManager.addPending(orderPtr);
    }

    bool hasPendingMission() const override {
        return m_missionManager.hasPending();
    }

    des::OrderPtr nextPendingMission() override {
        return m_missionManager.peekPending();
    }

    des::OrderPtr popPendingMission() override {
        return m_missionManager.popPending();
    }

    void addBackgroundMission(const des::OrderPtr orderPtr) override {
        m_missionManager.addBackground(orderPtr);
    }

    bool hasBackgroundMission() const override {
        return m_missionManager.hasBackground();
    }

    des::OrderPtr peekBackgroundMission() override {
        return m_missionManager.peekBackground();
    }

    des::OrderPtr popBackgroundMission() override {
        return m_missionManager.popBackground();
    }

    bool removeBackgroundMission(int orderId) override {
        return m_missionManager.removeBackground(orderId);
    }

    const std::vector<des::OrderPtr>& backgroundMissions() const override {
        return m_missionManager.background();
    }

    des::OrderPtr acceptFeasibleBackgroundMission() override {
        return m_missionManager.acceptFeasibleBackground(*this);
    }

    std::optional<int> getNextScheduledDispatchTime() const override {
        return m_queue.nextDispatchTime();
    }

    des::OrderPtr peekNextScheduledOrder() const override;

    std::optional<int> getSimulationEndTime() const override {
        return m_queue.nextEventTime(des::EventType::SIMULATION_END);
    }

    void publishMission(const des::OrderPtr& order, const int time) override {
        m_eventBus.notifyMissionPublished(order, time);
    }

    void publishMissionRegistered(const des::OrderPtr& order) override {
        m_eventBus.notifyMissionRegistered(order);
    }

    void advanceTime(const int newTime) {
        assert(newTime >= m_currentTime);
        m_robot->m_bat->updateBalance(newTime, m_robot->getState()->getEnergyConsumption(*this));
        if (m_robot->m_bat->isBatteryLow()) {
            m_robot->setChargingRequired(true);
        }
        m_currentTime = newTime;
        des::log::setSimTime(newTime);
    }

    // Observer functions (delegated to EventBus)
    void addObserver(const std::shared_ptr<IObserver>& observer) {
        m_eventBus.addObserver(observer);
    }

    void removeObserver(const std::shared_ptr<IObserver>& observer) {
        m_eventBus.removeObserver(observer);
    }

    void clearObservers() {
        m_eventBus.clear();
    }

    void notifyMissionComplete(const des::MissionState& state, const int timeDiff, const des::ExecutionMode execution) const {
        m_eventBus.notifyMissionComplete(m_currentTime, state, timeDiff, execution);
    }

    void notifyRobotStateChanged() const {
        const auto* state = m_robot->getState();
        m_eventBus.notifyStateChanged(m_currentTime, state->getType(), state->getName(), m_robot->m_bat->getStats());
    }

    void notifyBatteryChanged() const override {
        notifyRobotStateChanged();
    }

    void notifyEvent(const IEvent& event) const override {
        int missionId = event.getMissionId();
        if (missionId < 0) {
            if (const auto current = m_missionManager.getCurrent()) {
                missionId = current->id;
            }
        }
        m_eventBus.notifyEvent(event.time, event.getType(), event.getName(),
                               m_robot->isDriving(), m_robot->isCharging(),
                               event.getColor(), missionId);
    }

    void robotMoved(const std::string& location, const double distance = 0) const override {
        m_robot->setLocation(location);
        m_eventBus.notifyMoved(m_currentTime, location, distance);
    }

    double getDriveTimeStd() const { return m_simConfig->driveTimeStd; };


    bool pushInterrupt(const des::OrderPtr& order) override {
        if (!m_missionManager.pushInterrupt(order)) {
            return false;
        }

        // Snapshots robot state, shifts the in-flight activity-end event by the interrupt's duration
        InterruptSnapshot snap;
        snap.state = m_robot->getState()->clone();
        snap.wasDriving = m_robot->isDriving();

        auto& plugin = OrderRegistry::instance().get(order->type);
        const int duration = static_cast<int>(plugin.estimateMissionDuration(*order, *this, m_robot->getLocation()));

        if (auto e = m_robot->inFlight().lock()) {
            const int oldTime = e->time;
            const int newTime = oldTime + duration;
            m_queue.reschedule(e, newTime);
            DES_LOG_INFO(rclcpp::get_logger("des.context.interrupt"),
                        "Push %d (type=%s, dur=%ds) at t=%d — shifted in-flight '%s': %d → %d",
                        order->id, order->type.c_str(), duration, m_currentTime,
                        e->getName().c_str(), oldTime, newTime);
        } else {
            DES_LOG_INFO(rclcpp::get_logger("des.context.interrupt"),
                        "Push %d (type=%s, dur=%ds) at t=%d — robot idle, no in-flight to shift",
                        order->id, order->type.c_str(), duration, m_currentTime);
        }

        if (snap.wasDriving) {
            m_robot->setDriving(false);
            m_eventBus.notifyEvent(m_currentTime, des::EventType::STOP_DRIVE,
                                   "Drive paused: " + m_robot->getLocation(),
                                   false, m_robot->isCharging(), "", order->id);
        }

        m_interruptSnapshot = std::move(snap);
        plugin.onMissionStart(*this, *order);
        return true;
    }

    void popInterrupt(const des::OrderPtr& completedOrder) override {
        m_missionManager.popInterrupt(completedOrder);
        bool resumeDrive = false;
        if (m_interruptSnapshot) {
            auto snap = std::move(*m_interruptSnapshot);
            m_interruptSnapshot.reset();
            changeRobotState(std::move(snap.state));
            resumeDrive = snap.wasDriving;
        }
        if (resumeDrive) {
            m_robot->setDriving(true);
            m_eventBus.notifyEvent(m_currentTime, des::EventType::START_DRIVE,
                                   "Drive resumed: " + m_robot->getTargetLocation(),
                                   true, m_robot->isCharging(), "", -1);
        }
        DES_LOG_INFO(rclcpp::get_logger("des.context.interrupt"),
                    "Pop %d (type=%s) at t=%d — resuming main mission",
                    completedOrder->id, completedOrder->type.c_str(), m_currentTime);
    }

    bool hasActiveInterrupt() const override {
        return m_missionManager.hasInterrupt();
    }

};
