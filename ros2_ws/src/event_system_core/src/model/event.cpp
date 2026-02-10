#include <memory>

#include "robot_state.h"
#include "event.h"
#include "context.h"
#include "../util/rnd.h"


void SimulationStartEvent::execute(SimulationContext& ctx) {
    if (ctx.m_robot->isBusy()) { 
        ctx.changeRobotState(std::make_unique<IdleState>());
    }
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.notifyEvent(*this);
}

void SimulationEndEvent::execute(SimulationContext& ctx) {
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.notifyEvent(*this);
}

void FoundPersonConversationCompleteEvent::execute(SimulationContext& ctx) {
    bool successful = rnd::uni() < ctx.getConversationProbability();
    if(successful) {
        ctx.m_queue.push(std::make_shared<StartAccompanyEvent>(this->time));
    } else {
        ctx.updateAppointmentState(des::MissionState::FAILED);
        ctx.changeRobotState(std::make_unique<IdleState>());
        ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(this->time));
    }
    ctx.notifyEvent(*this);
}

void DropOffConversationCompleteEvent::execute(SimulationContext& ctx) {

    if(rnd::uni() < ctx.getConversationProbability()) {
        ctx.updateAppointmentState(des::MissionState::COMPLETED);
    } else {
        ctx.updateAppointmentState(des::MissionState::FAILED);
    }
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(time));
    ctx.notifyEvent(*this);
}

void StartDriveEvent::execute(SimulationContext& ctx) {
    if (ctx.m_robot->getLocation() == location) {
        ctx.m_queue.push(std::make_shared<StopDriveEvent>(time, location, 0));
    }
    Journey jrny = ctx.scheduleArrival(location);
    ctx.m_queue.push(std::make_shared<StopDriveEvent>(static_cast<int>(time + jrny.duration), location, jrny.distance));
    ctx.m_robot->setDriving(true);
    ctx.notifyEvent(*this);
}

void StopDriveEvent::execute(SimulationContext& ctx) {
    ctx.notifyEvent(*this);
    ctx.robotMoved(this->location, this->distance);
    ctx.m_robot->setDriving(false);
    ctx.m_behaviorTree->rootBlackboard()->set("location", this->location);
    ctx.m_behaviorTree->tickOnce();
}

void MissionDispatchEvent::execute(SimulationContext& ctx) {
    if (ctx.m_robot->isBusy()) {
        ctx.updateAppointmentState(des::MissionState::CANCELLED);
        this->appointment->state = des::MissionState::FAILED;
        ctx.notifyEvent(*this);
        return; 
    }

    ctx.setAppointment(this->appointment);
    ctx.updateAppointmentState(des::MissionState::IN_PROGRESS);
    std::string person = this->appointment->personName;

    assert(ctx.m_employeeLocations.find(person) != ctx.m_employeeLocations.end());
    std::vector<std::string> locations = ctx.m_employeeLocations.at(person);
    assert(!locations.empty());
    std::string firstLocation = locations.front();
    locations.erase(locations.begin());

    ctx.changeRobotState(std::make_unique<SearchState>(locations));
    ctx.m_queue.push(std::make_shared<StartDriveEvent>(time, firstLocation));
    ctx.notifyEvent(*this);
}

void AbortSearchEvent::execute(SimulationContext& ctx) {
    ctx.updateAppointmentState(des::MissionState::FAILED);
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(this->time));
    ctx.notifyEvent(*this);
}

void StartDropOffConversationeEvent::execute(SimulationContext& ctx) {
    double eventTime = this->time + ctx.getRndConversationTime();
    ctx.m_queue.push(std::make_shared<DropOffConversationCompleteEvent>(eventTime));
    ctx.notifyEvent(*this);
}

void StartFoundPersonConversationEvent::execute(SimulationContext& ctx) {
    ctx.m_robot->setDriving(false);
    auto eventTime = this->time + ctx.getRndConversationTime();
    ctx.m_queue.push(std::make_shared<FoundPersonConversationCompleteEvent>(eventTime));
    ctx.changeRobotState(std::make_unique<ConversateState>());
    ctx.notifyEvent(*this);
}

void StartAccompanyEvent::execute(SimulationContext& ctx) {
    ctx.changeRobotState(std::make_unique<AccompanyState>());
    ctx.m_queue.push(std::make_shared<StartDriveEvent>(time, ctx.getAppointment()->roomName));
    ctx.notifyEvent(*this);
}

void MissionCompleteEvent::execute(SimulationContext& ctx) {
    ctx.completeAppointment();
    ctx.m_behaviorTree->tickOnce();
    ctx.notifyEvent(*this);
}
