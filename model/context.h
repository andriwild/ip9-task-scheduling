#pragma once

#include <cassert>
#include <optional>
#include <vector>

#include "robot.h"
#include "event.h"
#include "../util/types.h"
#include "../observer/observer.h"
#include "../sim/ros/travel_time_est.h"


class SimulationContext {
    int currentTime = 0;
    std::vector<std::shared_ptr<IObserver>> observers;
    std::optional<des::Appointment> currentAppointment = std::nullopt;

public:
    Robot& robot;
    EventQueue& queue;
    std::shared_ptr<TravelTimeEstimator> travelTime;
    std::map<std::string, std::vector<std::string>> employeeLocations;

    explicit SimulationContext(
        Robot& robot, 
        EventQueue& queue,
        std::shared_ptr<TravelTimeEstimator> travelTime,
        std::map<std::string, std::vector<std::string>> employeeLocations
    ):
        robot(robot),
        queue(queue),
        travelTime(travelTime),
        employeeLocations(employeeLocations)
    {}

    void setAppointment(const des::Appointment& appt) {
        currentAppointment = appt;
    }

    void resetAppointment() {
        currentAppointment = std::nullopt;
    }

    std::optional<des::Appointment> getAppointment() const {
        return currentAppointment;
    }

    void setTime(int newTime) {
        assert(newTime > currentTime);
        currentTime = newTime;
    }

    void addObserver(std::shared_ptr<IObserver> observer) {
        observers.emplace_back(observer);
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
};
