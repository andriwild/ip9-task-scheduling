#pragma once

#include <vector>
#include <memory>
#include <cassert>
#include <optional>

#include "../model/event.h"
#include "../model/context.h"


class Scheduler {
public:
    explicit Scheduler(
        std::shared_ptr<SimulationContext> ctx,
        std::map<std::string, std::vector<std::string>> locations,
        rclcpp::Logger logger
    ):
        m_ctx(std::move(ctx)),
        m_locations(std::move(locations)),
        m_logger(logger)
    {}

    std::vector<std::shared_ptr<MissionDispatchEvent>> simplePlan(
        std::vector<std::shared_ptr<des::Appointment>>& appointments,
        bool useCache
    ) {
        RCLCPP_DEBUG(m_logger, "[SimplePlan] Schedule %zu appointments", appointments.size());
        std::vector<std::shared_ptr<MissionDispatchEvent>> missions;

        for (auto& appt : appointments) {

            assert(m_locations.find(appt->personName) != m_locations.end());
            auto employeeLocation = m_locations[appt->personName].front();

            auto startPos  = m_ctx->m_robot->getIdleLocation();
            auto targetPos = appt->roomName;

            std::optional<double> searchDist    = m_ctx->m_plannerNode->calcDistance(startPos, employeeLocation, useCache);
            std::optional<double> accompanyDist = m_ctx->m_plannerNode->calcDistance(employeeLocation, targetPos, useCache);

            assert(searchDist.has_value());
            assert(accompanyDist.has_value());

            const double travelTime    = searchDist.value() / m_ctx->m_robot->getDriveSpeed();
            const double accompanyTime = accompanyDist.value() / m_ctx->m_robot->getAccompanySpeed();
            const int startSeconds     = appt.get()->appointmentTime - (travelTime + accompanyTime);
            missions.emplace_back(std::make_shared<MissionDispatchEvent>(startSeconds, appt));
        }
        return missions;
    }

private:
    std::shared_ptr<SimulationContext> m_ctx;
    std::map<std::string, std::vector<std::string>> m_locations;
    rclcpp::Logger m_logger;
};
