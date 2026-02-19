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
    const auto result = rnd::uni() < ctx.getConversationProbability() ? des::Result::SUCCESS : des::Result::FAILURE;
    ctx.m_robot->getState()->setResult(result);
    ctx.m_behaviorTree->tickOnce();
    ctx.notifyEvent(*this);
}

void DropOffConversationCompleteEvent::execute(SimulationContext& ctx) {
    const auto result = rnd::uni() < ctx.getConversationProbability() ? des::Result::SUCCESS : des::Result::FAILURE;
    ctx.m_robot->getState()->setResult(result);
    ctx.m_behaviorTree->tickOnce();
    ctx.notifyEvent(*this);
}

void StartDriveEvent::execute(SimulationContext& ctx) {
    assert(!ctx.m_robot->isDriving());

    if (ctx.m_robot->getLocation() == location) {
        ctx.m_queue.push(std::make_shared<StopDriveEvent>(time, location, 0));
    }
    auto [duration, distance] = ctx.scheduleArrival(location);
    ctx.m_queue.push(std::make_shared<StopDriveEvent>(static_cast<int>(time + duration), location, distance));
    ctx.m_robot->setDriving(true);
    ctx.m_robot->setTargetLocation(location);
    ctx.notifyEvent(*this);
}

void StopDriveEvent::execute(SimulationContext& ctx) {
    ctx.robotMoved(this->location, this->distance);
    ctx.m_robot->setDriving(false);
    ctx.m_behaviorTree->rootBlackboard()->set("location", this->location);
    ctx.m_behaviorTree->tickOnce();
    ctx.notifyEvent(*this);
}

void AbortSearchEvent::execute(SimulationContext& ctx) {
    ctx.updateAppointmentState(des::MissionState::FAILED);
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(this->time, ctx.getAppointment()));
    ctx.notifyEvent(*this);
}

void StartDropOffConversationEvent::execute(SimulationContext& ctx) {
    double eventTime = this->time + ctx.getRndConversationTime();
    ctx.m_queue.push(std::make_shared<DropOffConversationCompleteEvent>(eventTime));
    ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::DROP_OFF));
    ctx.notifyEvent(*this);
}

void StartFoundPersonConversationEvent::execute(SimulationContext& ctx) {
    ctx.m_robot->setDriving(false);
    auto eventTime = this->time + ctx.getRndConversationTime();
    ctx.m_queue.push(std::make_shared<FoundPersonConversationCompleteEvent>(eventTime));
    ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));
    ctx.notifyEvent(*this);
}

void StartAccompanyEvent::execute(SimulationContext& ctx) {
    ctx.changeRobotState(std::make_unique<AccompanyState>());
    ctx.m_queue.push(std::make_shared<StartDriveEvent>(time, ctx.getAppointment()->roomName));
    ctx.notifyEvent(*this);
}

void MissionDispatchEvent::execute(SimulationContext& ctx) {
    ctx.addPendingMission(this->appointment);
    ctx.m_behaviorTree->tickOnce();
    ctx.notifyEvent(*this);
}

void MissionCompleteEvent::execute(SimulationContext& ctx) {
    ctx.completeAppointment();
    if (ctx.m_robot->updateAndGetChargingRequired()) {
        ctx.changeRobotState(std::make_unique<ChargeState>());
    }
    ctx.m_behaviorTree->tickOnce();
    ctx.notifyEvent(*this);
}

void BatteryFullEvent::execute(SimulationContext& ctx) {
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.m_behaviorTree->tickOnce();
    ctx.notifyEvent(*this);
}
