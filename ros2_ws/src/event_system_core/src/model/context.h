#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cassert>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "../observer/event_bus.h"
#include "../sim/i_path_planner.h"
#include "../util/types.h"
#include "event.h"
#include "i_sim_context.h"
#include "mission_manager.h"
#include "robot.h"
#include "../sim/scheduler.h"
#include "robot_state.h"
#include "../model/event_queue.h"


struct Journey {
    double duration;
    double distance;
};

class SimulationContext : public ISimContext {
    int m_currentTime{};
    std::shared_ptr<des::SimConfig> m_simConfig;
    EventBus m_eventBus;
    MissionManager m_missions;
    EventQueue& m_queue;
    std::shared_ptr<BT::Tree> m_behaviorTree;
    std::map<std::string, std::shared_ptr<des::Person>> m_employeeLocations;
    std::map<std::string, std::string> m_personLocations;
    std::unique_ptr<Scheduler> m_scheduler;

public:
    static constexpr unsigned int DEFAULT_SEED = 42;
    // mutable: RNG state changes are an implementation detail, allowing use in const methods
    mutable std::mt19937 m_rng{DEFAULT_SEED};

    std::shared_ptr<IPathPlanner> m_plannerNode;
    std::shared_ptr<Robot> m_robot;

    explicit SimulationContext(
        EventQueue& queue,
        std::shared_ptr<des::SimConfig> simConfig,
        std::shared_ptr<IPathPlanner> plannerNode,
        std::map<std::string, std::shared_ptr<des::Person>> employeeLocations
    );

    Scheduler& getScheduler() { return *m_scheduler; }
    const Scheduler& getScheduler() const { return *m_scheduler; }

    void resetRobot() {
        m_robot.reset();
        m_robot = std::make_unique<Robot>(m_simConfig);
    }

    Journey scheduleArrival(const std::string& target) const override;
    void changeRobotState(std::unique_ptr<RobotState> newState) const override;
    double getRndConversationTime() const override;
    void setConfig(const std::shared_ptr<des::SimConfig> &newConfig);
    void resetContext(int newTime);
    void completeAppointment(const std::shared_ptr<des::Appointment>& appt) const override;

    int getTime() const override { return m_currentTime; }

    std::shared_ptr<des::SimConfig> getConfig() const override { return m_simConfig; }

    std::shared_ptr<Robot> getRobot() const override { return m_robot; }

    std::mt19937& rng() const override { return m_rng; }

    // Event queue access
    void pushEvent(const std::shared_ptr<IEvent>& event) override {
        m_queue.push(event);
    }

    void extendEvents(const SortedEventQueue& events) {
        m_queue.extend(events);
    }

    int getFirstEventTime() const { return m_queue.getFirstEventTime(); }
    int getLastEventTime() const { return m_queue.getLastEventTime(); }

    std::shared_ptr<IEvent> topEvent() const { return m_queue.top(); }

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

    // Mission management (delegated to MissionManager)
    void setAppointment(const std::shared_ptr<des::Appointment>& appointment) override {
        m_missions.setCurrent(appointment);
    }

    std::shared_ptr<des::Appointment> getAppointment() const override {
        return m_missions.getCurrent();
    }

    void updateAppointmentState(const des::MissionState& newState) override {
        m_missions.updateState(newState);
    }

    void addPendingMission(const std::shared_ptr<des::Appointment>& appointment) override {
        m_missions.addPending(appointment);
    }

    bool hasPendingMission() const override {
        return m_missions.hasPending();
    }

    std::shared_ptr<des::Appointment> nextPendingMission() override {
        return m_missions.peekPending();
    }

    std::shared_ptr<des::Appointment> popPendingMission() override {
        return m_missions.popPending();
    }

    void advanceTime(const int newTime) {
        assert(newTime >= m_currentTime);
        m_robot->m_bat->updateBalance(newTime, m_robot->getState()->getEnergyConsumption(*this));
        if (m_robot->m_bat->isBatteryLow()) {
            m_robot->setChargingRequired(true);
        }
        m_currentTime = newTime;
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

    void notifyMissionComplete(const des::MissionState& state, const int timeDiff) const {
        m_eventBus.notifyMissionComplete(m_currentTime, state, timeDiff);
    }

    void notifyRobotStateChanged(const des::RobotStateType newState) const {
        m_eventBus.notifyStateChanged(m_currentTime, newState, m_robot->m_bat->getStats());
    }

    void notifyEvent(const IEvent& event) const override {
        m_eventBus.notifyEvent(event.time, event.getType(), event.getName(), m_robot->isDriving(), m_robot->isCharging(), event.getColor());
    }

    void robotMoved(const std::string& location, const double distance = 0) const override {
        m_robot->setLocation(location);
        m_eventBus.notifyMoved(m_currentTime, location, distance);
    }

    bool isMissionFeasible(const des::Appointment& appointment, const std::string &startPos) const override {
        const double missionDuration = m_scheduler->optimisticMeeting(appointment.personName, startPos, appointment.roomName);
        if (appointment.appointmentTime - missionDuration  >= getTime()) {
            RCLCPP_DEBUG(rclcpp::get_logger("Context"), "Mission %u is feasible", appointment.id);
            return true;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("Context"), "Mission %u is NOT feasible", appointment.id);
        return false;
    };

    double getConversationProbability() const override { return m_simConfig->conversationProbability; };
    double getDefaultConversationTime() const { return m_simConfig->conversationDurationMean; };
    double getConversationDurationStd() const { return m_simConfig->conversationDurationStd; };
    double getDriveTimeStd() const { return m_simConfig->driveTimeStd; };
};
