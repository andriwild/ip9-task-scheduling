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
#include "robot_state.h"


class SimulationContext {
    int m_currentTime;
    std::shared_ptr<des::SimConfig> m_simConfig;
    std::vector<std::shared_ptr<IObserver>> m_observers;
    std::shared_ptr<des::Appointment> m_currentAppointment = nullptr;
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
        rclcpp::Logger logger
    );

    void scheduleArrival(int currentTime, const std::string target, bool isMissionComplete = false);
    void changeRobotState(std::unique_ptr<RobotState> newState);
    double getRndConversationTime();
    void setConfig(std::shared_ptr<des::SimConfig> newConfig);
    void updateAppointmentState(const des::MissionState& newState);
    void resetContext(int newTime);
    void completeAppointment();

    int getTime() const { return m_currentTime; };

    const std::shared_ptr<des::SimConfig> getConfig() const { return m_simConfig; };

    void setAppointment(std::shared_ptr<des::Appointment> appt) {
        m_currentAppointment = appt;
    }

    const std::shared_ptr<des::Appointment> getAppointment() const {
        return m_currentAppointment;
    }

    void setTime(int newTime) {
        assert(newTime >= m_currentTime);
        m_currentTime = newTime;
    }

    // Observer functions
    void addObserver(std::shared_ptr<IObserver> observer) {
        RCLCPP_INFO(rclcpp::get_logger("SimulationContext"), "Observer added: %s", observer->getName().c_str());
        m_observers.emplace_back(observer);
    }

    void notifyMissionComplete(des::MissionState state, int timeDiff) {
        for (auto obs : m_observers) {
            obs->onMissionComplete(m_currentTime, state, timeDiff);
        }
    }

    void notifyRobotStateChanged(des::RobotStateType newState) {
        for (auto obs : m_observers) {
            obs->onStateChanged(m_currentTime, newState, m_robot->m_bat->getStats());
        }
    }

    void notifyEvent(IEvent& event) {
        for (auto obs : m_observers) {
            obs->onEvent(event.time, event.getType(), event.getName());
        }
    }

    void robotMoved(std::string location, double distance = 0) {
        m_robot->setLocation(location);
        notifyMoved(location, distance);
    }

    void notifyMoved(std::string location, double distance) {
        for (auto obs : m_observers) {
            obs->onRobotMoved(m_currentTime, location, distance);
        }
    }

    double getPersonFindProbability() const { return m_simConfig->personFindProbability; };
    double getConversationProbability() const { return m_simConfig->conversationProbability; };
    double getDefaultConversationTime() const { return m_simConfig->conversationDurationMean; };
    double getConversationDurationStd() const { return m_simConfig->conversationDurationStd; };
    double getdriveTimeVariance() const { return m_simConfig->driveTimeStd; };
};
