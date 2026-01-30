#include <memory>

#include "robot_state.h"
#include "event.h"
#include "context.h"


void SimulationStartEvent::execute(SimulationContext& ctx) {
    if (ctx.m_robot->isBusy()) { 
        ctx.changeRobotState(std::make_unique<IdleState>());
    }
    ctx.notifyEvent(*this);
    ctx.changeRobotState(std::make_unique<IdleState>());
}

void SimulationEndEvent::execute(SimulationContext& ctx) {
    ctx.notifyEvent(*this);
    ctx.changeRobotState(std::make_unique<IdleState>());
}

void FoundPersonConversationCompleteEvent::execute(SimulationContext& ctx) {
    bool successful = rnd::uni() < ctx.getConversationProbability();
    ctx.notifyEvent(*this);
    if(successful) {
        ctx.m_queue.push(std::make_shared<StartAccompanyEvent>(this->time));
    } else {
        ctx.updateAppointmentState(des::MissionState::FAILED);
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(this->time));
    }
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
}

void DropOffConversationCompleteEvent::execute(SimulationContext& ctx) {
    bool successful = rnd::uni() < ctx.getConversationProbability();
    if(successful) {
        ctx.notifyEvent(*this);
        ctx.updateAppointmentState(des::MissionState::COMPLETED);
    } else {
        ctx.notifyEvent(*this);
        ctx.updateAppointmentState(des::MissionState::FAILED);
    }
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(this->time));
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
    ctx.m_behaviorTree->tickOnce();
}

void ArrivedEvent::execute(SimulationContext& ctx) {
    ctx.notifyEvent(*this);
    ctx.robotMoved(this->location, this->distance);
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
    ctx.m_behaviorTree->rootBlackboard()->set("location", this->location);
    ctx.m_behaviorTree->tickOnce();
}

void MissionDispatchEvent::execute(SimulationContext& ctx) {
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
    if (ctx.m_robot->isBusy()) {
        ctx.updateAppointmentState(des::MissionState::CANCELLED);
        this->appointment->state = des::MissionState::FAILED;
        ctx.notifyEvent(*this);
        return; 
    }

    ctx.setAppointment(this->appointment);
    ctx.updateAppointmentState(des::MissionState::IN_PROGRESS);
    std::string person = this->appointment->personName;

    if (ctx.m_employeeLocations.find(person) == ctx.m_employeeLocations.end()) {
        ctx.notifyEvent(*this);
        return;
    }

    std::vector<std::string> locations = ctx.m_employeeLocations.at(person);
    
    if (locations.empty()) {
        ctx.notifyEvent(*this);
        return;
    }

    std::string firstGoal = locations.front();
    locations.erase(locations.begin());

    ctx.notifyEvent(*this);
    
    ctx.changeRobotState(std::make_unique<SearchState>(locations));
    ctx.scheduleArrival(this->time, firstGoal);
}

void AbortSearchEvent::execute(SimulationContext& ctx) {
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
    ctx.updateAppointmentState(des::MissionState::FAILED);
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(this->time));
    ctx.notifyEvent(*this);
}

void StartDropOffConversationeEvent::execute(SimulationContext& ctx) {
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
    double eventTime = this->time + ctx.getRndConversationTime();
    ctx.m_queue.push(std::make_shared<DropOffConversationCompleteEvent>(eventTime));
    ctx.notifyEvent(*this);
}

void StartFoundPersonConversationEvent::execute(SimulationContext& ctx) {
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
    auto eventTime = this->time + ctx.getRndConversationTime();
    ctx.m_queue.push(std::make_shared<FoundPersonConversationCompleteEvent>(eventTime));
    ctx.changeRobotState(std::make_unique<ConversateState>());
    ctx.notifyEvent(*this);
}

void StartAccompanyEvent::execute(SimulationContext& ctx) {
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
    ctx.changeRobotState(std::make_unique<AccompanyState>());
    ctx.scheduleArrival(this->time, ctx.getAppointment()->roomName);
    ctx.notifyEvent(*this);
}

void MissionCompleteEvent::execute(SimulationContext& ctx) {
    ctx.m_behaviorTree->rootBlackboard()->set("current_time", this->time);
    ctx.completeAppointment();
    ctx.notifyEvent(*this);
}
