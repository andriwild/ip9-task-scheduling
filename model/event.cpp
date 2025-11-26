#include <memory>
#include <optional>
#include <iostream>

#include "robot_state.h"
#include "event.h"
#include "context.h"

const std::string NODE_DOCK = "Dock";

void SimulationStartEvent::execute(SimulationContext& ctx) {
    if (ctx.robot.isBusy()) { 
        ctx.robot.changeState(new IdleState());
    }
    ctx.notifyLog("Simulation started");
    
    ctx.robot.moveTo(NODE_DOCK);
    ctx.notifyMoved(NODE_DOCK);
}

void SimulationEndEvent::execute(SimulationContext& ctx) {
    ctx.notifyLog("Simulation ended");
}

void ArrivedEvent::execute(SimulationContext& ctx) {
    ctx.robot.moveTo(this->location);
    ctx.notifyMoved(ctx.robot.getLocation());

    // 2. Logik basierend auf Zustand
    // BESSER: ctx.robot.getState()->handleArrival(ctx); 
    // Aber hier ist die optimierte Version deiner Logik:

    if (ctx.robot.isSearching()) {
        // Simulation des Findens (Hier Wahrscheinlichkeit einbauen!)
        bool personFound = true; // TODO: std::bernoulli_distribution(0.5)(ctx.rng);

        if (personFound) {
            if (!ctx.getAppointment().has_value()) {
                ctx.notifyLog("[CRITICAL] Searching without appointment data!");
                return;
            }

            ctx.notifyLog("Person found! Starting escort.");
            auto goal = ctx.getAppointment().value().roomName;
            
            ctx.robot.changeState(new EscortState());
            ctx.scheduleArrival(this->time, goal, true); 

        } else {
            ctx.notifyLog("Person not in " + this->location + ". Checking next room...");
            
            // WICHTIG: Der SearchState muss wissen, welche Räume noch übrig sind.
            // Das erfordert Zugriff auf den State. 
            // auto* searchState = dynamic_cast<SearchState*>(ctx.robot.getCurrentState());
            // std::string nextRoom = searchState->popNextRoom();
            // ctx.scheduleArrival(this->time, nextRoom);
        }

    } else if (ctx.robot.isEscorting()) {
        ctx.notifyLog("Escort finished. Returning to Dock.");
        ctx.robot.changeState(new MoveState());
        ctx.scheduleArrival(this->time, NODE_DOCK);

    } else {
        ctx.notifyLog("Robot is idle at " + this->location);
        ctx.robot.changeState(new IdleState());
    }
}

void MissionDispatchEvent::execute(SimulationContext& ctx) {
    if (ctx.robot.isBusy()) {
        ctx.notifyLog("[WARN] Robot busy, ignoring dispatch.");
        return; 
    }

    ctx.setAppointment(this->appointment);
    std::string person = this->appointment.personName;

    if (ctx.employeeLocations.find(person) == ctx.employeeLocations.end()) {
        ctx.notifyLog("[ERROR] Person '" + person + "' not found in database!");
        return;
    }

    std::vector<std::string> locations = ctx.employeeLocations.at(person);
    
    if (locations.empty()) {
        ctx.notifyLog("[ERROR] No locations defined for '" + person + "'!");
        return;
    }

    std::string firstGoal = locations.front();
    locations.erase(locations.begin());

    ctx.notifyLog("Mission Dispatch: Searching for " + person);
    
    ctx.robot.changeState(new SearchState(locations));
    ctx.scheduleArrival(this->time, firstGoal);
}

void MissionCompleteEvent::execute(SimulationContext& ctx) {
    ctx.notifyLog("Mission Complete. Appointment cleared.");
    ctx.resetAppointment();
}
