#pragma once

#include <utility>
#include <vector>
#include <memory>
#include <cassert>
#include <optional>
#include <map>
#include <rclcpp/rclcpp.hpp>

#include "../model/event.h"
#include "../sim/i_path_planner.h"

class Scheduler {
    std::shared_ptr<des::SimConfig> m_simConfig;
    std::shared_ptr<IPathPlanner> m_plannerNode;
    const std::map<std::string, std::shared_ptr<des::Person>>& m_locations;
    const des::SearchAreaMap& m_searchAreas;

public:
    explicit Scheduler(
        const std::shared_ptr<des::SimConfig> &simConfig,
        const std::shared_ptr<IPathPlanner> &plannerNode,
        const std::map<std::string, std::shared_ptr<des::Person>>& locations,
        const des::SearchAreaMap& searchAreas
    )
        : m_simConfig(simConfig)
        , m_plannerNode(plannerNode)
        , m_locations(locations)
        , m_searchAreas(searchAreas)
    {}

    std::vector<std::shared_ptr<MissionDispatchEvent>> simplePlan(
        std::vector<std::shared_ptr<des::Appointment>>& appointments,
        const std::string& startPos
    ) {
        RCLCPP_DEBUG(rclcpp::get_logger("Scheduler"), "[SimplePlan] Schedule %zu appointments", appointments.size());
        std::vector<std::shared_ptr<MissionDispatchEvent>> missions;

        for (auto& appointment : appointments) {
            assert(m_locations.contains(appointment->personName));
            const double driveTime = pessimisticMeeting(appointment->personName, startPos, appointment->roomName);
            const double startSeconds = appointment->appointmentTime - static_cast<int>(driveTime) - m_simConfig->timeBuffer;
            missions.emplace_back(std::make_shared<MissionDispatchEvent>(startSeconds, appointment));
        }
        return missions;
    }

    // Calc time to accompany a person to a meeting with using only one search location
    double optimisticMeeting(const std::string& personName, const std::string& startPos, const std::string& goalPos) {
        const auto employeeLocation = m_locations.at(personName)->roomLabels.front();
        const double searchTime     = getDriveTime(startPos, employeeLocation, m_simConfig->robotSpeed);
        const double scanTime       = getScanTime(employeeLocation);
        const double accompanyTime  = getDriveTime(employeeLocation, goalPos, m_simConfig->robotAccompanySpeed);
        return searchTime + accompanyTime + scanTime;
    }

    // Calc time to accompany a person to a meeting with using all the search locations
    double pessimisticMeeting(const std::string& personName, const std::string& startPos, const std::string& goalPos) {
        double searchTime = 0.0;
        std::string currentPos = startPos;
        for (const auto& location: m_locations.at(personName)->roomLabels) {
            searchTime += getDriveTime(currentPos, location, m_simConfig->robotSpeed);
            currentPos = location;
        }
        const double accompanyTime = getDriveTime(currentPos, goalPos, m_simConfig->robotAccompanySpeed);
        return searchTime + accompanyTime;
    }

private:
    [[nodiscard]] double getDriveTime(const std::string& startPos, const std::string& goalPos, const double speed) const {
        const std::optional<double> dist = m_plannerNode->calcDistance(startPos, goalPos, m_simConfig->cacheEnabled);
        assert(dist.has_value());
        return dist.value() / speed;
    }

    [[nodiscard]] double getScanTime(const std::string& area) const {
        auto it = m_searchAreas.find(area);
        if (it == m_searchAreas.end()) {
            RCLCPP_WARN(rclcpp::get_logger("Scheduler"), "Search area not found for '%s', defaulting to 1.0", area.c_str());
            return 1.0;
        }
        return it->second;
    }
};
