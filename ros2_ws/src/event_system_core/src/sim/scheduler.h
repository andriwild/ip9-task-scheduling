#pragma once

#include <utility>
#include <vector>
#include <memory>
#include <cassert>
#include <optional>
#include <map>
#include <rclcpp/rclcpp.hpp>

#include "../model/event.h"
#include "../sim/ros/path_node.h"

class Scheduler {
    std::shared_ptr<des::SimConfig> m_simConfig;
    std::shared_ptr<PathPlannerNode> m_plannerNode;
    std::map<std::string, std::vector<std::string>> m_locations;

public:
    explicit Scheduler(
        const std::shared_ptr<des::SimConfig> &simConfig,
        const std::shared_ptr<PathPlannerNode> &plannerNode,
        std::map<std::string, std::vector<std::string>> locations
    )
        : m_simConfig(simConfig)
        , m_plannerNode(plannerNode)
        , m_locations(std::move(locations))
    {}

    std::vector<std::shared_ptr<MissionDispatchEvent>> simplePlan(
        std::vector<std::shared_ptr<des::Appointment>>& appointments,
        const std::string& startPos
    ) {
        RCLCPP_DEBUG(rclcpp::get_logger("Scheduler"), "[SimplePlan] Schedule %zu appointments", appointments.size());
        std::vector<std::shared_ptr<MissionDispatchEvent>> missions;

        for (auto& appointment : appointments) {
            assert(m_locations.contains(appointment->personName));
            const double driveTime = optimisticMeeting(appointment->personName, startPos, appointment->roomName);
            const double startSeconds = appointment->appointmentTime - static_cast<int>(driveTime) - m_simConfig->timeBuffer;
            missions.emplace_back(std::make_shared<MissionDispatchEvent>(startSeconds, appointment));
        }
        return missions;
    }

    // Calc time to accompany a person to a meeting with using only one search location
    double optimisticMeeting(const std::string& personName, const std::string& startPos, const std::string& goalPos) {
        const auto employeeLocation = m_locations[personName].front();
        const double searchTime    = driveTime(startPos, employeeLocation, m_simConfig->robotSpeed);
        const double accompanyTime = driveTime(employeeLocation, goalPos, m_simConfig->robotAccompanySpeed);
        return searchTime + accompanyTime;
    }

    // Calc time to accompany a person to a meeting with using all the search locations
    double pessimisticMeeting(const std::string& personName, const std::string& startPos, const std::string& goalPos) {
        double searchTime = 0.0;
        std::string currentPos = startPos;
        for (const auto& location: m_locations[personName]) {
            searchTime += driveTime(currentPos, location, m_simConfig->robotSpeed);
            currentPos = location;
        }
        const double accompanyTime = driveTime(currentPos, goalPos, m_simConfig->robotAccompanySpeed);
        return searchTime + accompanyTime;
    }

private:
    [[nodiscard]] double driveTime(const std::string& startPos, const std::string& goalPos, const double speed) const {
        const std::optional<double> dist = m_plannerNode->calcDistance(startPos, goalPos, m_simConfig->cacheEnabled);
        assert(dist.has_value());
        return dist.value() / speed;
    }
};
