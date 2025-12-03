#pragma once

#include <cassert>
#include <optional>
#include <vector>
#include <memory>
#include <behaviortree_cpp/bt_factory.h>

#include "robot.h"
#include "event.h"
#include "../util/types.h"
#include "../observer/observer.h"
#include "../sim/ros/travel_time_est.h"
#include "robot_state.h"


class SimulationContext {
    int currentTime = 0;
    double personDetectionProbability;
    std::vector<std::shared_ptr<IObserver>> observers;
    std::optional<des::Appointment*> currentAppointment = std::nullopt;

public:
    Robot& robot;
    EventQueue& queue;
    std::shared_ptr<TravelTimeEstimator> travelTime;
    std::map<std::string, std::vector<std::string>> employeeLocations;
    std::shared_ptr<BT::Tree> behaviorTree;

    explicit SimulationContext(
        Robot& robot, 
        EventQueue& queue,
        double personDetectionProbability,
        std::shared_ptr<TravelTimeEstimator> travelTime,
        std::map<std::string, std::vector<std::string>> employeeLocations
    ):
        robot(robot),
        queue(queue),
        travelTime(travelTime),
        employeeLocations(employeeLocations),
        personDetectionProbability(personDetectionProbability)
    {}

    void setAppointment(des::Appointment& appt) {
        currentAppointment = &appt;
    }

    void completeAppointment() {
        if(currentTime > currentAppointment.value()->appointmentTime) {
            currentAppointment.value()->state = des::AppointmentState::COMPLETED_LATE;
        } else {
            currentAppointment.value()->state = des::AppointmentState::COMPLETED;
        }
        currentAppointment = std::nullopt;
    }

    std::optional<des::Appointment*> getAppointment() const {
        return currentAppointment;
    }

    void updateAppointmentState(const des::AppointmentState& newState) {
        currentAppointment.value()->state = newState;
    }

    void setTime(int newTime) {
        assert(newTime > currentTime);
        currentTime = newTime;
    }

    void addObserver(std::shared_ptr<IObserver> observer) {
        observers.emplace_back(observer);
    }

    void changeRobotState(RobotState* newState) {
        robot.changeState(newState);
        notifyRobotStateChanged(newState->getType());
    }

    void notifyRobotStateChanged(RobotStateType newState) {
        for (auto obs: observers){
            obs->onStateChanged(currentTime, newState);
        }
    }

    void notifyLog(const std::string& msg) {
        for (auto obs: observers){
            obs->onLog(currentTime, msg);
        }
    }

    void notifyMoved(std::string location) {
        for (auto obs:observers ) {
            obs->onRobotMoved(currentTime, location);
        }
    }

    double getPersonDetectionProbability() const { return personDetectionProbability; };

    void scheduleArrival(int currentTime, const std::string& target, bool isMissionComplete = false) {
        const bool needsMoving = robot.getLocation() != target;

        std::optional<double> duration = 1;
        if (needsMoving) {
            duration = this->travelTime->estimateDuration(
                this->robot.getLocation(), 
                target,
                this->robot.getSpeed()
            );
        }

        if (duration.has_value()) {
            int arrivalTime = currentTime + duration.value(); // TODO: Add uncertainty here
            this->queue.push(std::make_shared<ArrivedEvent>(arrivalTime, target));

            if (isMissionComplete) {
                this->queue.push(std::make_shared<MissionCompleteEvent>(arrivalTime + 1));
            }
            this->notifyLog("Moving to " + target + " (" + std::to_string(duration.value()) + "s)");
        } else {
            this->notifyLog("[ERROR] Path planning failed to " + target);
        }
    }
};
