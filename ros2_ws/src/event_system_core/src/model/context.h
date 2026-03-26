#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cassert>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "../observer/event_bus.h"
#include "../sim/ros/path_node.h"
#include "../util/types.h"
#include "event.h"
#include "robot.h"
#include "../sim/scheduler.h"
#include "robot_state.h"
#include "../model/event_queue.h"


struct Journey {
    double duration;
    double distance;
};

class SimulationContext {
    int m_currentTime{};
    std::shared_ptr<des::SimConfig> m_simConfig;
    EventBus m_eventBus;
    std::shared_ptr<des::Appointment> m_currentAppointment = nullptr;
    std::queue<std::shared_ptr<des::Appointment>> m_pendingMissions;

public:
    static constexpr unsigned int DEFAULT_SEED = 42;
    mutable std::mt19937 m_rng{DEFAULT_SEED};

    std::shared_ptr<PathPlannerNode> m_plannerNode;
    std::shared_ptr<Robot> m_robot;
    EventQueue& m_queue;
    std::shared_ptr<BT::Tree> m_behaviorTree;
    std::map<std::string, std::vector<std::string>> m_employeeLocations;
    Scheduler& m_scheduler;

    explicit SimulationContext(
        EventQueue& queue,
        std::shared_ptr<des::SimConfig> simConfig,
        std::shared_ptr<PathPlannerNode> plannerNode,
        std::map<std::string, std::vector<std::string>> employeeLocations,
        Scheduler& scheduler
    );

    void resetRobot() {
        m_robot.reset();
        m_robot = std::make_unique<Robot>(m_simConfig);
    }

    Journey scheduleArrival(const std::string& target) const;
    void changeRobotState(std::unique_ptr<RobotState> newState) const;
    double getRndConversationTime() const;
    void setConfig(const std::shared_ptr<des::SimConfig> &newConfig);
    void updateAppointmentState(const des::MissionState& newState) const;
    void resetContext(int newTime);
    void completeAppointment(const std::shared_ptr<des::Appointment>& appt) const;

    int getTime() const { return m_currentTime; }

    std::shared_ptr<des::SimConfig> getConfig() const { return m_simConfig; }

    void setAppointment(const std::shared_ptr<des::Appointment> &appointment) {
        m_currentAppointment = appointment;
    }

    void addPendingMission(const std::shared_ptr<des::Appointment>& appointment) {
        m_pendingMissions.push(appointment);
        RCLCPP_DEBUG(rclcpp::get_logger("Context"), "Mission added to pending list - queue size: %zu", m_pendingMissions.size());
    }
    
    bool hasPendingMission() const {
        return !m_pendingMissions.empty();
    }

    std::shared_ptr<des::Appointment> nextPendingMission() {
        if (m_pendingMissions.empty()) { return nullptr; }
        return m_pendingMissions.front();
    }
    
    std::shared_ptr<des::Appointment> popPendingMission() {
        if (m_pendingMissions.empty()) { return nullptr; }
        auto appointment = m_pendingMissions.front();
        m_pendingMissions.pop();
        RCLCPP_DEBUG(rclcpp::get_logger("Context"), "Mission removed from pending list - %zu remaining", m_pendingMissions.size());
        return appointment;
    }

    std::shared_ptr<des::Appointment> getAppointment() const {
        return m_currentAppointment;
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

    void notifyEvent(const IEvent& event) const {
        m_eventBus.notifyEvent(event.time, event.getType(), event.getName(), m_robot->isDriving(), m_robot->isCharging());
    }

    void robotMoved(const std::string& location, const double distance = 0) const {
        m_robot->setLocation(location);
        m_eventBus.notifyMoved(m_currentTime, location, distance);
    }

    bool isMissionFeasible(const des::Appointment& appointment, const std::string &startPos) const {
        const double missionDuration = m_scheduler.optimisticMeeting(appointment.personName, startPos, appointment.roomName);
        if (appointment.appointmentTime - missionDuration  >= getTime()) {
            RCLCPP_DEBUG(rclcpp::get_logger("Context"), "Mission %u is feasible", appointment.id);
            return true;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("Context"), "Mission %u is NOT feasible", appointment.id);
        return false;
    };

    double getPersonFindProbability() const { return m_simConfig->personFindProbability; };
    double getConversationProbability() const { return m_simConfig->conversationProbability; };
    double getDefaultConversationTime() const { return m_simConfig->conversationDurationMean; };
    double getConversationDurationStd() const { return m_simConfig->conversationDurationStd; };
    double getDriveTimeStd() const { return m_simConfig->driveTimeStd; };
};
