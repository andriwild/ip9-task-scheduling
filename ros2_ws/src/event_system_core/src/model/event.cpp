#include <memory>

#include "robot_state.h"
#include "event.h"
#include "context.h"


void SimulationStartEvent::execute(SimulationContext& ctx) {
    if (ctx.m_robot->isBusy()) { 
        ctx.changeRobotState(std::make_unique<IdleState>());
    }
    ctx.notifyEvent("Simulation started");
    ctx.changeRobotState(std::make_unique<IdleState>());
}

void SimulationEndEvent::execute(SimulationContext& ctx) {
    ctx.notifyEvent("Simulation ended");
    ctx.changeRobotState(std::make_unique<IdleState>());
}

void FoundPersonConversationCompleteEvent::execute(SimulationContext& ctx) {
    bool successful = rnd::uni() < ctx.getConversationProbability();
    if(successful) {
        ctx.notifyEvent("Conversation ended successful. Person for accompany convinced.");
        ctx.changeRobotState(std::make_unique<AccompanyState>());
        ctx.scheduleArrival(this->time, ctx.getAppointment()->roomName);
    } else {
        ctx.notifyEvent("Conversation failed.");
        ctx.updateAppointmentState(des::MissionState::FAILED);
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(this->time + 1));
    }
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
}

void DropOffConversationCompleteEvent::execute(SimulationContext& ctx) {
    bool successful = rnd::uni() < ctx.getConversationProbability();
    if(successful) {
        ctx.notifyEvent("Conversation ended successful. Person dropped off.");
        ctx.updateAppointmentState(des::MissionState::COMPLETED);
    } else {
        ctx.notifyEvent("Conversation failed.");
        ctx.updateAppointmentState(des::MissionState::FAILED);
    }
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(this->time + 1));
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
    ctx.m_behaviorTree->tickOnce();
}

void ArrivedEvent::execute(SimulationContext& ctx) {
    ctx.robotMoved(this->location, this->distance);
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
    ctx.m_behaviorTree->rootBlackboard()->set("location", this->location);
    ctx.m_behaviorTree->tickOnce();
}

void MissionDispatchEvent::execute(SimulationContext& ctx) {
    if (ctx.m_robot->isBusy()) {
        ctx.updateAppointmentState(des::MissionState::CANCELLED);
        this->appointment->state = des::MissionState::FAILED;
        ctx.notifyEvent("[WARN] Robot busy, ignoring dispatch.");
        return; 
    }

    ctx.setAppointment(this->appointment);
    ctx.updateAppointmentState(des::MissionState::IN_PROGRESS);
    std::string person = this->appointment->personName;

    if (ctx.m_employeeLocations.find(person) == ctx.m_employeeLocations.end()) {
        ctx.notifyEvent("[ERROR] Person '" + person + "' not found in database!");
        return;
    }

    std::vector<std::string> locations = ctx.m_employeeLocations.at(person);
    
    if (locations.empty()) {
        ctx.notifyEvent("[ERROR] No locations defined for '" + person + "'!");
        return;
    }

    std::string firstGoal = locations.front();
    locations.erase(locations.begin());

    ctx.notifyEvent("Mission Dispatch: Searching for " + person);
    
    ctx.changeRobotState(std::make_unique<SearchState>(locations));
    ctx.scheduleArrival(this->time, firstGoal);
}

void MissionCompleteEvent::execute(SimulationContext& ctx) {
    ctx.notifyEvent("Mission Complete. Appointment cleared.");
    ctx.completeAppointment();
}
