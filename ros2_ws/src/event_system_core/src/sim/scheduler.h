#pragma once

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
    rclcpp::Logger m_logger;

public:
    explicit Scheduler(
        std::shared_ptr<des::SimConfig> simConfig,
        std::shared_ptr<PathPlannerNode> plannerNode,
        std::map<std::string, std::vector<std::string>> locations,
        rclcpp::Logger logger
    )
        : m_simConfig(simConfig)
        , m_plannerNode(plannerNode)
        , m_locations(std::move(locations))
        , m_logger(logger)
    {}

    std::vector<std::shared_ptr<MissionDispatchEvent>> simplePlan(
        std::vector<std::shared_ptr<des::Appointment>>& appointments,
        std::string startPos
    ) {
        RCLCPP_DEBUG(m_logger, "[SimplePlan] Schedule %zu appointments", appointments.size());
        std::vector<std::shared_ptr<MissionDispatchEvent>> missions;

        for (auto& appt : appointments) {
            assert(m_locations.find(appt->personName) != m_locations.end());
            int startSeconds = calcStartTime(*appt, startPos);
            missions.emplace_back(std::make_shared<MissionDispatchEvent>(startSeconds, appt));
        }
        return missions;
    }

    int calcStartTime(des::Appointment& appt, std::string& startPos) {
        auto employeeLocation = m_locations[appt.personName].front();

        auto targetPos = appt.roomName;

        std::optional<double> searchDist    = m_plannerNode->calcDistance(startPos, employeeLocation, m_simConfig->cacheEnabled);
        std::optional<double> accompanyDist = m_plannerNode->calcDistance(employeeLocation, targetPos, m_simConfig->cacheEnabled);

        assert(searchDist.has_value());
        assert(accompanyDist.has_value());

        const double travelTime    = searchDist.value() / m_simConfig->robotSpeed;
        const double accompanyTime = accompanyDist.value() / m_simConfig->robotAccompanySpeed;
        return appt.appointmentTime - (travelTime + accompanyTime);
    }

};
