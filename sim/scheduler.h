#pragma once

#include <vector>
#include <memory>
#include <cassert>
#include <optional>

#include "../model/event.h"
#include "../model/context.h"
#include "../view/timeline.h"

inline void scheduleAppointments(
    std::vector<std::shared_ptr<des::Appointment>> appointments,
    SimulationContext& ctx,
    Timeline& timelineView,
    std::map<std::string, std::vector<std::string>>& locations
) {
    for (auto& appt : appointments) {
        auto employeeLocation = locations[appt->personName].front();

        auto startPos = ctx.robot.getIdleLocation();
        double speed  = ctx.robot.getDefaultSpeed();
        
        std::optional<double> travelTo   = ctx.travelTime->estimateDistance(startPos, employeeLocation);
        std::optional<double> escorting  = ctx.travelTime->estimateDistance(employeeLocation, appt.get()->roomName);
        std::optional<double> travelBack = ctx.travelTime->estimateDistance(appt.get()->roomName, startPos);

        assert(travelTo.has_value());
        assert(escorting.has_value());
        assert(travelBack.has_value());
        
        double taskOverhead = 30;
        double buffer = 60;
        
        const double accTravelDist =  travelTo.value() + escorting.value() + travelBack.value();
        const double travelTime = accTravelDist / ctx.robot.getDefaultSpeed();
        const int startSeconds = appt.get()->appointmentTime - (travelTime + taskOverhead + buffer);

        ctx.queue.push(std::make_shared<MissionDispatchEvent>(startSeconds, appt));
        timelineView.addMeetingPlan(appt, startSeconds);
    }
}
