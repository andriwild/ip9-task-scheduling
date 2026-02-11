#include <memory>

#include "robot_state.h"
#include "event.h"
#include "context.h"
#include "../util/rnd.h"


void SimulationStartEvent::execute(SimulationContext& ctx) {
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.notifyEvent(*this);
}

void SimulationEndEvent::execute(SimulationContext& ctx) {
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.notifyEvent(*this);
}

void FoundPersonConversationCompleteEvent::execute(SimulationContext& ctx) {
    ctx.m_behaviorTree->tickOnce();
    ctx.notifyEvent(*this);
}

void DropOffConversationCompleteEvent::execute(SimulationContext& ctx) {
    bool successful = rnd::uni() < ctx.getConversationProbability();
    auto currentState = ctx.m_robot->getState();
    auto convState = dynamic_cast<ConversateState*>(currentState);
    convState->complete(successful);
    ctx.m_behaviorTree->tickOnce();
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
    ctx.robotMoved(this->location, this->distance);
    ctx.m_robot->setDriving(false);
    ctx.m_behaviorTree->rootBlackboard()->set("location", this->location);
    ctx.m_behaviorTree->tickOnce();
    ctx.notifyEvent(*this);
}

void MissionDispatchEvent::execute(SimulationContext& ctx) {
    // Just add to queue, decisions are made in Behavior Tree
    ctx.addPendingMission(this->appointment);
    ctx.m_behaviorTree->tickOnce();
    ctx.notifyEvent(*this);
}

void AbortSearchEvent::execute(SimulationContext& ctx) {
    ctx.updateAppointmentState(des::MissionState::FAILED);
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(this->time, ctx.getAppointment()));
    ctx.notifyEvent(*this);
}

void StartDropOffConversationeEvent::execute(SimulationContext& ctx) {
    double eventTime = this->time + ctx.getRndConversationTime();
    bool successful = rnd::uni() < ctx.getConversationProbability();
    ctx.m_queue.push(std::make_shared<DropOffConversationCompleteEvent>(eventTime, successful));
    ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::DROP_OFF));
    ctx.notifyEvent(*this);
}

void StartFoundPersonConversationEvent::execute(SimulationContext& ctx) {
    ctx.m_robot->setDriving(false);
    auto eventTime = this->time + ctx.getRndConversationTime();
    bool successful = rnd::uni() < ctx.getConversationProbability();
    ctx.m_queue.push(std::make_shared<FoundPersonConversationCompleteEvent>(eventTime, successful));
    ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));
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
