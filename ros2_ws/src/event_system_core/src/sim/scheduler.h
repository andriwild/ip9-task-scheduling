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
        auto employeeLocation = locations[appt->personName].front();

        auto startPos = ctx->robot->getIdleLocation();
        
        std::optional<double> travelTo   = ctx->travelTime->estimateDistance(startPos, employeeLocation);
        std::optional<double> escorting  = ctx->travelTime->estimateDistance(employeeLocation, appt.get()->roomName);
        std::optional<double> travelBack = ctx->travelTime->estimateDistance(appt.get()->roomName, startPos);

        assert(travelTo.has_value());
        assert(escorting.has_value());
        assert(travelBack.has_value());
        
        const double accTravelDist =  travelTo.value() + escorting.value() + travelBack.value();
        const double travelTime = accTravelDist / ctx->robot->getDefaultSpeed();
        const int startSeconds = appt.get()->appointmentTime - travelTime;
        missions.emplace_back(std::make_shared<MissionDispatchEvent>(startSeconds, appt));
    }
    return missions;
}
