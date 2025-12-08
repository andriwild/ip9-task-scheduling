#pragma once

#include <cassert>
#include <vector>
#include <memory>
#include <behaviortree_cpp/bt_factory.h>

#include "robot.h"
#include "event.h"
#include "robot_state.h"
#include "../util/types.h"
#include "../util/rnd.h"
#include "../observer/observer.h"
#include "../sim/ros/path_planner.h"


class SimulationContext {
    int currentTime = 0;
    double personDetectionProbability;
    double driveTimeVariance;
    std::vector<std::shared_ptr<IObserver>> observers;
    des::Appointment* currentAppointment = nullptr;

public:
    Robot& robot;
    EventQueue& queue;
    std::shared_ptr<PathPlanner> travelTime;
    std::map<std::string, std::vector<std::string>> employeeLocations;
    std::shared_ptr<BT::Tree> behaviorTree;

    explicit SimulationContext(
        Robot& robot, 
        EventQueue& queue,
        double personDetectionProbability,
        double driveTimeVariance,
        std::shared_ptr<PathPlanner> travelTime,
        std::map<std::string, std::vector<std::string>> employeeLocations
    ):
        robot(robot),
        queue(queue),
        travelTime(travelTime),
        employeeLocations(employeeLocations),
        personDetectionProbability(personDetectionProbability),
        driveTimeVariance(driveTimeVariance)
    {}

    void setAppointment(des::Appointment& appt) {
        currentAppointment = &appt;
    }

    void completeAppointment() {
        assert(currentAppointment != nullptr);
        int  timeDiff = currentTime - currentAppointment->appointmentTime;
        notifyMissionComplete(currentAppointment->state, timeDiff);
        currentAppointment = nullptr;
    }

    const des::Appointment* getAppointment() const {
        return currentAppointment;
    }

    void updateAppointmentState(const des::MissionState& newState) {
        assert(currentAppointment != nullptr);
        currentAppointment->state = newState;
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

    void notifyMissionComplete(des::MissionState state, int timeDiff) {
        for (auto obs: observers){
            obs->onMissionComplete(currentTime, state, timeDiff);
        }
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

    void notifyMoved(std::string location, double distance) {
        for (auto obs:observers ) {
            obs->onRobotMoved(currentTime, location, distance);
        }
    }

    double getPersonDetectionProbability() const { return personDetectionProbability; };

    double getdriveTimeVariance() const { return driveTimeVariance; };

    void scheduleArrival(int currentTime, const std::string target, bool isMissionComplete = false) {
        int arrivalTime = currentTime + 1;

        if(robot.getLocation() == target){
            this->queue.push(std::make_shared<ArrivedEvent>(currentTime + 1, target, 0));
            this->notifyLog("Already at " + target);
        } else {
            std::optional<double> distance = this->travelTime->estimateDistance( this->robot.getLocation(), target);
            assert(distance.has_value());

            double travelTime = distance.value() / this->robot.getDefaultSpeed();

            double noiseFactor = rnd::getNormalDist(1.0, 0.1);
            arrivalTime = currentTime + (travelTime * noiseFactor);
            
            this->queue.push(std::make_shared<ArrivedEvent>(arrivalTime, target, distance.value()));
            this->notifyLog("Moving to " + target + " (" + std::to_string(travelTime) + "s)");
        }

        if (isMissionComplete) {
            this->queue.push(std::make_shared<MissionCompleteEvent>(arrivalTime + 1));
        }
    }
};
