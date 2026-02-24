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

void StartDriveEvent::execute(SimulationContext& ctx) {
    assert(!ctx.m_robot->isDriving());
    ctx.m_robot->m_bat->updateBalance(ctx.getTime(), ctx.m_robot->getState()->getEnergyConsumption(ctx));
    ctx.m_robot->setCharging(false);

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
    ctx.m_robot->m_bat->updateBalance(ctx.getTime(), ctx.m_robot->getState()->getEnergyConsumption(ctx));
    ctx.m_robot->setDriving(false);
    if (ctx.m_robot->getLocation() == ctx.m_robot->getIdleLocation()) {
        ctx.m_robot->setCharging(true);
    }
    ctx.m_behaviorTree->rootBlackboard()->set("location", this->location);
    ctx.notifyEvent(*this);

    ctx.m_behaviorTree->tickOnce();
}

void AbortSearchEvent::execute(SimulationContext& ctx) {
    ctx.updateAppointmentState(des::MissionState::FAILED);
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.m_queue.push(std::make_shared<MissionCompleteEvent>(this->time, ctx.getAppointment()));
    ctx.notifyEvent(*this);
}

void StartDropOffConversationEvent::execute(SimulationContext& ctx) {
    assert(!ctx.m_robot->isDriving());
    double eventTime = this->time + ctx.getRndConversationTime();
    if (rnd::uni() < ctx.getConversationProbability()) {
        ctx.m_queue.push(std::make_shared<SuccessDropOffConversationCompleteEvent>(eventTime));
    } else {
        ctx.m_queue.push(std::make_shared<FailedDropOffConversationCompleteEvent>(eventTime));
    }
    ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::DROP_OFF));
    ctx.notifyEvent(*this);
}

void SuccessDropOffConversationCompleteEvent::execute(SimulationContext& ctx) {
    ctx.m_robot->getState()->setResult(des::Result::SUCCESS);
    ctx.notifyEvent(*this);
    ctx.m_behaviorTree->tickOnce();
}

void FailedDropOffConversationCompleteEvent::execute(SimulationContext& ctx) {
    ctx.m_robot->getState()->setResult(des::Result::FAILURE);
    ctx.notifyEvent(*this);
    ctx.m_behaviorTree->tickOnce();
}

void StartFoundPersonConversationEvent::execute(SimulationContext& ctx) {
    assert(!ctx.m_robot->isDriving());
    auto eventTime = this->time + ctx.getRndConversationTime();
    if (rnd::uni() < ctx.getConversationProbability()) {
        ctx.m_queue.push(std::make_shared<SuccessFoundPersonConversationCompleteEvent>(eventTime));
    } else {
        ctx.m_queue.push(std::make_shared<FailedFoundPersonConversationCompleteEvent>(eventTime));
    }
    ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));
    ctx.notifyEvent(*this);
}

void SuccessFoundPersonConversationCompleteEvent::execute(SimulationContext& ctx) {
    ctx.m_robot->getState()->setResult(des::Result::SUCCESS);
    ctx.notifyEvent(*this);
    ctx.m_behaviorTree->tickOnce();
}

void FailedFoundPersonConversationCompleteEvent::execute(SimulationContext& ctx) {
    ctx.m_robot->getState()->setResult(des::Result::FAILURE);
    ctx.notifyEvent(*this);
    ctx.m_behaviorTree->tickOnce();
}

void StartAccompanyEvent::execute(SimulationContext& ctx) {
    ctx.changeRobotState(std::make_unique<AccompanyState>());
    ctx.m_queue.push(std::make_shared<StartDriveEvent>(time, ctx.getAppointment()->roomName));
    ctx.notifyEvent(*this);
}

void MissionDispatchEvent::execute(SimulationContext& ctx) {
    ctx.addPendingMission(this->appointment);
    ctx.notifyEvent(*this);
    ctx.m_behaviorTree->tickOnce();
}

void MissionCompleteEvent::execute(SimulationContext& ctx) {
    ctx.completeAppointment(this->appointment);
    ctx.notifyEvent(*this);
    ctx.m_behaviorTree->tickOnce();
}

void BatteryFullEvent::execute(SimulationContext& ctx) {
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.m_robot->m_batteryFullEventScheduled = false;
    ctx.notifyEvent(*this);
    ctx.m_behaviorTree->tickOnce();
}

void MissionStartEvent::execute(SimulationContext& ctx) {
    const std::string person = appointment->personName;
    assert(ctx.m_employeeLocations.contains(person));
    std::vector<std::string> locations = ctx.m_employeeLocations.at(person);
    assert(!locations.empty());
    ctx.changeRobotState(std::make_unique<SearchState>(locations));
    ctx.notifyEvent(*this);
    ctx.m_behaviorTree->tickOnce();
}