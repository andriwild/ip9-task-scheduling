#include "robot_state.h"
#include "event.h"
#include "context.h"


void SimulationStartEvent::execute(SimulationContext& ctx) {
    if (ctx.robot.isBusy()) { 
        ctx.changeRobotState(new IdleState());
    }
    ctx.notifyLog("Simulation started");
    
    ctx.robot.setLocation(ctx.robot.getIdleLocation());
    ctx.notifyMoved(ctx.robot.getIdleLocation());
    ctx.changeRobotState(new IdleState);
}

void SimulationEndEvent::execute(SimulationContext& ctx) {
    ctx.notifyLog("Simulation ended");
    ctx.changeRobotState(new IdleState);
}

void ArrivedEvent::execute(SimulationContext& ctx) {
    ctx.robot.setLocation(this->location);
    ctx.notifyMoved(ctx.robot.getLocation());

    if (ctx.behaviorTree) {
        ctx.behaviorTree->rootBlackboard()->set("current_time", this->time);
        ctx.behaviorTree->rootBlackboard()->set("location", this->location);
        
        ctx.behaviorTree->tickOnce();
    } else {
        ctx.notifyLog("[FATAL] Behavior Tree not initialized in Context!");
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
    
    ctx.changeRobotState(new SearchState(locations));
    ctx.scheduleArrival(this->time, firstGoal);
}

void MissionCompleteEvent::execute(SimulationContext& ctx) {
    ctx.notifyLog("Mission Complete. Appointment cleared.");
    ctx.resetAppointment();
    ctx.changeRobotState(new IdleState());
}
