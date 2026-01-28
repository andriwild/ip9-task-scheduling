#pragma once

#include <vector>
#include <memory>
#include <cassert>
#include <optional>

#include "../model/event.h"
#include "../model/context.h"

inline std::vector<std::shared_ptr<MissionDispatchEvent>> scheduleAppointments(
    std::vector<std::shared_ptr<des::Appointment>>& appointments,
    std::map<std::string, std::vector<std::string>>& locations,
    std::shared_ptr<SimulationContext> ctx
) {
    std::vector<std::shared_ptr<MissionDispatchEvent>> missions;
    for (auto& appt : appointments) {

        assert(locations.find(appt->personName) != locations.end());
        auto employeeLocation = locations[appt->personName].front();

        auto startPos = ctx->m_robot->getIdleLocation();
        
        std::optional<double> travelTo  = ctx->m_plannerNode->estimateDistance(startPos, employeeLocation);
        std::optional<double> accompany = ctx->m_plannerNode->estimateDistance(employeeLocation, appt.get()->roomName);

        assert(travelTo.has_value());
        assert(accompany.has_value());
        
        const double travelTime    = travelTo.value() / ctx->m_robot->getSpeed();
        const double accompanyTime = accompany.value() / ctx->m_robot->getAccompanySpeed();
        const int startSeconds     = appt.get()->appointmentTime - (travelTime + accompanyTime);
        missions.emplace_back(std::make_shared<MissionDispatchEvent>(startSeconds, appt));
    }
    return missions;
}
