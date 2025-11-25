#include <memory>
#include <optional>

#include "robot_state.h"
#include "event.h"
#include "context.h"


void SimulationStartEvent::execute(SimulationContext& ctx) {
    if(ctx.robot.isBusy()) {
        ctx.robot.changeState(new IdleState());
    }
    ctx.notifyLog("Simulation start");
    ctx.robot.moveTo("Dock");
    ctx.notifyMoved("Dock");
}

void SimulationEndEvent::execute(SimulationContext& ctx) {
    if(ctx.robot.isBusy()) {
        ctx.robot.changeState(new IdleState());
    }
    ctx.notifyLog("Simulation end");
}

void ArrivedEvent::execute(SimulationContext& ctx) {
    ctx.robot.moveTo(this->location);
    ctx.notifyMoved(ctx.robot.getLocation());

    if(ctx.robot.isSearching()){
        // TODO: find not always person
        if(!ctx.getAppointment().has_value()){
            ctx.notifyLog("[ERROR] Searching without an appointment!");
        }
        auto goal = ctx.getAppointment().value().roomName;
        ctx.robot.changeState(new EscortState());

        auto duration = ctx.travelTime->estimateDuration(
            ctx.robot.getLocation(), 
            goal,
            ctx.robot.getSpeed()
        );

        if(duration.has_value()) {
            int estimatedArrival = this->time + duration.value(); // TODO: add uncertainty
            ctx.queue.push(std::make_shared<ArrivedEvent>(estimatedArrival, goal));
            ctx.queue.push(std::make_shared<MissionCompleteEvent>(estimatedArrival+1));
            ctx.notifyLog("Start escorting...");
        } else {
            ctx.notifyLog("[ERROR] Failed to calculate path to " + goal);
        }
    } else if(ctx.robot.isEscorting()) {
        std::string goal = "Dock";
        ctx.notifyLog("Escorting finished");

        ctx.robot.changeState(new MoveState());

        auto duration = ctx.travelTime->estimateDuration(
            ctx.robot.getLocation(), 
            goal,
            ctx.robot.getSpeed()
        );

        if(duration.has_value()) {
            int estimatedArrival = this->time + duration.value(); // TODO: add uncertainty
            ctx.queue.push(std::make_shared<ArrivedEvent>(estimatedArrival, goal));
            ctx.notifyLog("Moving ...");
        } else {
            ctx.notifyLog("[ERROR] Failed to calculate path to " + goal);
        }
    } else {
        ctx.robot.changeState(new IdleState());
    }
}

void MissionDispatchEvent::execute(SimulationContext& ctx) {
    if(!ctx.robot.isBusy()) {
        ctx.setAppointment(this->appointment);

        std::string person = this->appointment.personName;
        std::vector<std::string> locations = ctx.employeeLocations[person];
        std::string goal = locations.front();
        locations.erase(locations.begin());

        ctx.robot.changeState(new SearchState(locations));

        auto duration = ctx.travelTime->estimateDuration(
            ctx.robot.getLocation(), 
            goal,
            ctx.robot.getSpeed()
        );

        if(duration.has_value()) {
            int estimatedArrival = this->time + duration.value(); // TODO: add uncertainty
            ctx.queue.push(std::make_shared<ArrivedEvent>(estimatedArrival, goal));
            ctx.notifyLog("Mission Dispatch");
        } else {
            ctx.notifyLog("Failed to calculate path to " + goal);
        }
    }
}

void MissionCompleteEvent::execute(SimulationContext& ctx) {
    ctx.notifyLog("Mission Complete");
    ctx.resetAppointment();
}
