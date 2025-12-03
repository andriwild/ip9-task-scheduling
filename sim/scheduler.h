#pragma once

#include <vector>
#include <memory>
#include <cassert>
#include <optional>

#include "../model/event.h"
#include "../model/context.h"
#include "../view/timeline.h"

inline void scheduleAppointments(
    std::vector<des::Appointment>& appointments,
    SimulationContext& ctx,
    Timeline& timelineView,
    std::map<std::string, std::vector<std::string>>& locations
) {
    for (auto& appt : appointments) {
        auto employeeLocation = locations[appt.personName].front();

        auto startPos = ctx.robot.getIdleLocation();
        double speed  = ctx.robot.getDefaultSpeed();
        
        std::optional<double> travelTo   = ctx.travelTime->estimateDuration(startPos, employeeLocation, speed);
        std::optional<double> escorting  = ctx.travelTime->estimateDuration(employeeLocation, appt.roomName, speed);
        std::optional<double> travelBack = ctx.travelTime->estimateDuration(appt.roomName, startPos, speed);
        
        double taskOverhead = 30;
        double buffer = 60;
        
        assert(travelTo.has_value());
        assert(escorting.has_value());
        assert(travelBack.has_value());

        const double travelTime = travelTo.value() + escorting.value() + travelBack.value();
        const int startSeconds = appt.appointmentTime - (travelTime + taskOverhead + buffer);

        ctx.queue.push(std::make_shared<MissionDispatchEvent>(startSeconds, &appt));
        timelineView.addMeetingPlan(appt, startSeconds);
    }
}
