#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cassert>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "../observer/observer.h"
#include "../sim/ros/path_node.h"
#include "../util/types.h"
#include "event.h"
#include "robot.h"
#include "../sim/scheduler.h"
#include "robot_state.h"


struct Journey {
    double duration;
    double distance;
};


class SimulationContext {
    int m_currentTime{};
    std::shared_ptr<des::SimConfig> m_simConfig;
    std::vector<std::shared_ptr<IObserver>> m_observers;
    std::shared_ptr<des::Appointment> m_currentAppointment = nullptr;
    std::queue<std::shared_ptr<des::Appointment>> m_pendingMissions;
    Scheduler& m_scheduler;
    rclcpp::Logger m_logger;

public:
    std::shared_ptr<PathPlannerNode> m_plannerNode;
    std::shared_ptr<Robot> m_robot;
    EventQueue& m_queue;
    std::shared_ptr<BT::Tree> m_behaviorTree;
    std::map<std::string, std::vector<std::string>> m_employeeLocations;

    explicit SimulationContext(
        EventQueue& queue,
        std::shared_ptr<des::SimConfig> simConfig,
        std::shared_ptr<PathPlannerNode> plannerNode,
        std::map<std::string, std::vector<std::string>> employeeLocations,
        Scheduler& scheduler,
        const rclcpp::Logger& logger
    );

    Journey scheduleArrival(const std::string& target) const;
    void changeRobotState(std::unique_ptr<RobotState> newState);
    double getRndConversationTime();
    void setConfig(std::shared_ptr<des::SimConfig> newConfig);
    void updateAppointmentState(const des::MissionState& newState);
    void resetContext(int newTime);
    void completeAppointment();

    int getTime() const { return m_currentTime; };

    const std::shared_ptr<des::SimConfig> getConfig() const { return m_simConfig; };

    void setAppointment(const std::shared_ptr<des::Appointment> &appt) {
        m_currentAppointment = appt;
    }

    void addPendingMission(const std::shared_ptr<des::Appointment>& appt) {
        m_pendingMissions.push(appt);
        RCLCPP_DEBUG(m_logger, "Mission added to pending list - queue size: %zu", m_pendingMissions.size());
    }
    
    bool hasPendingMission() const {
        return !m_pendingMissions.empty();
    }

    std::shared_ptr<des::Appointment> nextPendingMission() {
        if (m_pendingMissions.empty()) {
            return nullptr;
        }
        return m_pendingMissions.front();
    }
    
    std::shared_ptr<des::Appointment> popPendingMission() {
        if (m_pendingMissions.empty()) {
            return nullptr;
        }
        auto appt = m_pendingMissions.front();
        m_pendingMissions.pop();
        RCLCPP_DEBUG(m_logger, "Mission removed from pending list - %zu remaining", m_pendingMissions.size());
        return appt;
    }

    const std::shared_ptr<des::Appointment> getAppointment() const {
        return m_currentAppointment;
    }

    void setTime(const int newTime) {
        assert(newTime >= m_currentTime);
        m_currentTime = newTime;
    }

    // Observer functions
    void addObserver(const std::shared_ptr<IObserver>& observer) {
        RCLCPP_INFO(rclcpp::get_logger("SimulationContext"), "Observer added: %s", observer->getName().c_str());
        m_observers.emplace_back(observer);
    }

    void notifyMissionComplete(des::MissionState& state, int timeDiff) const {
        for (const auto& obs : m_observers) {
            obs->onMissionComplete(m_currentTime, state, timeDiff);
        }
    }

    void notifyRobotStateChanged(const des::RobotStateType newState) const {
        for (const auto& obs : m_observers) {
            obs->onStateChanged(m_currentTime, newState, m_robot->m_bat->getStats());
        }
    }

    void notifyEvent(const IEvent& event) const {
        for (const auto& obs : m_observers) {
            obs->onEvent(event.time, event.getType(), event.getName(), m_robot->isDriving());
        }
    }

    void robotMoved(const std::string& location, double distance = 0) const {
        m_robot->setLocation(location);
        notifyMoved(location, distance);
    }

    void notifyMoved(const std::string& location, const double distance) const {
        for (const auto& obs : m_observers) {
            obs->onRobotMoved(m_currentTime, location, distance);
        }
    }

    bool isMissionFeasible(des::Appointment& appt, std::string startPos) {
        const int startTime = m_scheduler.calcStartTime(appt, startPos);
        if (startTime + m_simConfig->timeBuffer >= getTime()) {
            RCLCPP_DEBUG(m_logger, "Mission %u is feasible", appt.id);
            return true;
        }
        RCLCPP_DEBUG(m_logger, "Mission %u is NOT feasible", appt.id);
        return false;
    };

    double getPersonFindProbability() const { return m_simConfig->personFindProbability; };
    double getConversationProbability() const { return m_simConfig->conversationProbability; };
    double getDefaultConversationTime() const { return m_simConfig->conversationDurationMean; };
    double getConversationDurationStd() const { return m_simConfig->conversationDurationStd; };
    double getDriveTimeVariance() const { return m_simConfig->driveTimeStd; };
};
