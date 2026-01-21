#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cassert>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "../observer/observer.h"
#include "../sim/ros/path_node.h"
#include "../util/rnd.h"
#include "../util/types.h"
#include "event.h"
#include "robot.h"
#include "robot_state.h"

class SimulationContext {
    int m_currentTime = 0;
    std::shared_ptr<des::SimConfig> m_simConfig;
    std::vector<std::shared_ptr<IObserver>> m_observers;
    std::shared_ptr<des::Appointment> m_currentAppointment = nullptr;

public:
    std::shared_ptr<Robot> m_robot;
    EventQueue& m_queue;
    std::shared_ptr<PathPlannerNode> m_plannerNode;
    std::shared_ptr<BT::Tree> m_behaviorTree;
    std::map<std::string, std::vector<std::string>> m_employeeLocations;

    explicit SimulationContext(
        EventQueue& queue,
        std::shared_ptr<des::SimConfig> simConfig,
        std::shared_ptr<PathPlannerNode> plannerNode,
        std::map<std::string, std::vector<std::string>> employeeLocations 
    ):
        m_simConfig(simConfig),
        m_queue(queue),
        m_plannerNode(plannerNode),
        m_employeeLocations(employeeLocations)
    {
        m_robot = std::make_unique<Robot>(simConfig->robotSpeed, simConfig->robotAccompanySpeed);
        RCLCPP_INFO(rclcpp::get_logger("SimulationContext"), "Simulation Context created!");
    }

    void setAppointment(std::shared_ptr<des::Appointment> appt) {
        m_currentAppointment = appt;
    }

    void completeAppointment() {
        assert(m_currentAppointment != nullptr);
        int timeDiff = m_currentTime - m_currentAppointment->appointmentTime;
        notifyMissionComplete(m_currentAppointment->state, timeDiff);
    }

    const std::shared_ptr<des::Appointment> getAppointment() const {
        return m_currentAppointment;
    }

    void updateAppointmentState(const des::MissionState& newState) {
        assert(m_currentAppointment != nullptr);
        m_currentAppointment->state = newState;
    }

    void resetTime(int newTime) {
        m_currentTime = newTime;
    }

    void setTime(int newTime) {
        assert(newTime >= m_currentTime);
        m_currentTime = newTime;
    }

    void addObserver(std::shared_ptr<IObserver> observer) {
        RCLCPP_INFO(rclcpp::get_logger("SimulationContext"), "Observer added: %s", observer->getName().c_str());
        m_observers.emplace_back(observer);
    }

    void changeRobotState(std::unique_ptr<RobotState> newState) {
        m_robot->changeState(std::move(newState));
        notifyRobotStateChanged(m_robot->getState()->getType());
    }

    void notifyMissionComplete(des::MissionState state, int timeDiff) {
        for (auto obs : m_observers) {
            obs->onMissionComplete(m_currentTime, state, timeDiff);
        }
    }

    void notifyRobotStateChanged(RobotStateType newState) {
        for (auto obs : m_observers) {
            obs->onStateChanged(m_currentTime, newState);
        }
    }

    void notifyLog(const std::string& msg) {
        for (auto obs : m_observers) {
            obs->onLog(m_currentTime, msg);
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

    void setConfig(std::shared_ptr<des::SimConfig> newConfig) {
        m_simConfig = newConfig;
        m_robot->setSpeed(m_simConfig->robotSpeed);
        m_robot->setAccompanytSpeed(m_simConfig->robotAccompanySpeed);
        RCLCPP_INFO(rclcpp::get_logger("SimulationContext"), "New config active");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("SimulationContext"), *m_simConfig);
    }

    double getPersonFindProbability() const { return m_simConfig->personFindProbability; };
    double getConversationFoundStd() const { return m_simConfig->conversationFoundStd; };
    double getConversationDropOffStd() const { return m_simConfig->conversationDropOffStd; };
    double getdriveTimeVariance() const { return m_simConfig->driveStd; };

    void scheduleArrival(int currentTime, const std::string target, bool isMissionComplete = false) {
        int arrivalTime = currentTime + 1;

        if (m_robot->getLocation() == target) {
            this->m_queue.push(std::make_shared<ArrivedEvent>(currentTime + 1, target, 0));
            this->notifyLog("Already at " + target);
        } else {
            std::optional<double> distance = this->m_plannerNode->estimateDistance(this->m_robot->getLocation(), target);
            assert(distance.has_value());

            double travelTime = distance.value() / this->m_robot->getSpeed();

            double noiseFactor = rnd::getNormalDist(1.0, 0.1);
            double completeTravelTime = travelTime * noiseFactor;
            arrivalTime = currentTime + completeTravelTime;

            this->m_queue.push(std::make_shared<ArrivedEvent>(arrivalTime, target, distance.value()));
            this->notifyLog("Moving to " + target + " (" + std::to_string(travelTime) + "s + " + std::to_string(completeTravelTime - travelTime) + ")");
        }

        if (isMissionComplete) {
            this->m_queue.push(std::make_shared<MissionCompleteEvent>(arrivalTime + 1));
        }
    }
};
