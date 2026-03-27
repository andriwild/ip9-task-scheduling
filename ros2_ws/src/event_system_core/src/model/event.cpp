#include <iterator>
#include <memory>

#include <cassert>
#include "robot_state.h"
#include "robot.h"
#include "event.h"
#include "context.h"
#include "../util/rnd.h"

void SimulationStartEvent::execute(ISimContext& ctx) {
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.notifyEvent(*this);
    ctx.pushEvent(std::make_shared<StopDriveEvent>(time, ctx.getRobot()->getLocation(), 0));
}

void SimulationEndEvent::execute(ISimContext& ctx) {
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.notifyEvent(*this);
    ctx.pushEvent(std::make_shared<StopDriveEvent>(time, ctx.getRobot()->getLocation(), 0));
}

void StartDriveEvent::execute(ISimContext& ctx) {
    assert(!ctx.getRobot()->isDriving());
    ctx.getRobot()->setCharging(false);

    if (ctx.getRobot()->getLocation() == location) {
        ctx.pushEvent(std::make_shared<StopDriveEvent>(time, location, 0));
    }
    auto [duration, distance] = ctx.scheduleArrival(location);
    ctx.pushEvent(std::make_shared<StopDriveEvent>(static_cast<int>(time + duration), location, distance));
    ctx.getRobot()->setDriving(true);
    ctx.getRobot()->setTargetLocation(location);
    ctx.notifyEvent(*this);
}

void StopDriveEvent::execute(ISimContext& ctx) {
    ctx.robotMoved(this->location, this->distance);
    ctx.getRobot()->setDriving(false);
    ctx.setBTBlackboard("location", this->location);
    ctx.notifyEvent(*this);

    // Move accompanied person to arrival location
    if (ctx.getRobot()->getStateType() == des::RobotStateType::ACCOMPANY && ctx.getAppointment()) {
        const auto& personName = ctx.getAppointment()->personName;
        if (ctx.hasEmployee(personName)) {
            auto person = ctx.getPersonByName(personName);
            person->currentRoom = ctx.getRobot()->getLocation();
            ctx.pushEvent(std::make_shared<PersonTransitionEvent>(this->time, person));
        }
    }

    ctx.tickBT();
}

void AbortSearchEvent::execute(ISimContext& ctx) {
    ctx.updateAppointmentState(des::MissionState::FAILED);
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.pushEvent(std::make_shared<MissionCompleteEvent>(this->time, ctx.getAppointment()));
    ctx.notifyEvent(*this);
}

void StartDropOffConversationEvent::execute(ISimContext& ctx) {
    assert(!ctx.getRobot()->isDriving());
    double eventTime = this->time + ctx.getRndConversationTime();
    if (rnd::uni(ctx.rng()) < ctx.getConversationProbability()) {
        ctx.pushEvent(std::make_shared<SuccessDropOffConversationCompleteEvent>(eventTime));
    } else {
        ctx.pushEvent(std::make_shared<FailedDropOffConversationCompleteEvent>(eventTime));
    }
    ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::DROP_OFF));
    ctx.notifyEvent(*this);
}

void SuccessDropOffConversationCompleteEvent::execute(ISimContext& ctx) {
    ctx.getRobot()->getState()->setResult(des::Result::SUCCESS);
    ctx.notifyEvent(*this);
    ctx.tickBT();
}

void FailedDropOffConversationCompleteEvent::execute(ISimContext& ctx) {
    ctx.getRobot()->getState()->setResult(des::Result::FAILURE);
    ctx.notifyEvent(*this);
    ctx.tickBT();
}

void StartFoundPersonConversationEvent::execute(ISimContext& ctx) {
    assert(!ctx.getRobot()->isDriving());
    auto eventTime = this->time + ctx.getRndConversationTime();
    if (rnd::uni(ctx.rng()) < ctx.getConversationProbability()) {
        ctx.pushEvent(std::make_shared<SuccessFoundPersonConversationCompleteEvent>(eventTime));
    } else {
        ctx.pushEvent(std::make_shared<FailedFoundPersonConversationCompleteEvent>(eventTime));
    }
    ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));
    ctx.notifyEvent(*this);
}

void SuccessFoundPersonConversationCompleteEvent::execute(ISimContext& ctx) {
    ctx.getRobot()->getState()->setResult(des::Result::SUCCESS);
    ctx.notifyEvent(*this);
    ctx.tickBT();
}

void FailedFoundPersonConversationCompleteEvent::execute(ISimContext& ctx) {
    ctx.getRobot()->getState()->setResult(des::Result::FAILURE);
    ctx.notifyEvent(*this);
    ctx.tickBT();
}

void StartAccompanyEvent::execute(ISimContext& ctx) {
    ctx.changeRobotState(std::make_unique<AccompanyState>());
    ctx.pushEvent(std::make_shared<StartDriveEvent>(time, ctx.getAppointment()->roomName));
    ctx.notifyEvent(*this);
}

void MissionDispatchEvent::execute(ISimContext& ctx) {
    ctx.addPendingMission(this->appointment);
    ctx.notifyEvent(*this);
    ctx.tickBT();
}

void MissionCompleteEvent::execute(ISimContext& ctx) {
    ctx.completeAppointment(this->appointment);
    ctx.notifyEvent(*this);
    ctx.tickBT();
}

void BatteryFullEvent::execute(ISimContext& ctx) {
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.getRobot()->m_batteryFullEventScheduled = false;
    ctx.notifyEvent(*this);
    ctx.tickBT();
}

void PersonTransitionEvent::execute(ISimContext& ctx) {
    auto& p = *this->person;
    if (p.currentRoom == "OUTDOOR") {
        ctx.notifyEvent(*this);
        return;
    }

    auto it = std::find(p.roomLabels.begin(), p.roomLabels.end(), p.currentRoom);

    // Person was moved outside their known rooms (e.g. by accompany) — notify, then return to workplace
    if (it == p.roomLabels.end()) {
        ctx.notifyEvent(*this);
        auto returnCopy = std::make_shared<des::Person>(p);
        returnCopy->currentRoom = p.workplace;
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(
            this->time + rnd::uni(ctx.rng(), 60, ONE_HOUR), returnCopy));
        return;
    }

    int currentIndex = std::distance(p.roomLabels.begin(), it);
    const std::vector<double>& row = p.transitionMatrix.at(currentIndex);
    int nextRoomIdx = rnd::discrete_dist(ctx.rng(), row);

    p.currentRoom = p.roomLabels.at(nextRoomIdx);
    ctx.notifyEvent(*this);

    double nextExecutionTime;
    des::RoomType roomType = des::parseRoomName(p.currentRoom);
    switch (roomType) {
        case des::RoomType::WORKPLACE:
            nextExecutionTime = this->time + rnd::uni(ctx.rng(), 60 * 10, ONE_HOUR * 2);
            break;
        case des::RoomType::TOILET:
            nextExecutionTime = this->time + rnd::logNormal(ctx.rng(), 4.8, 0.7);
            break;
        case des::RoomType::KITCHEN:
            nextExecutionTime = this->time + rnd::uni(ctx.rng(), 30, 1800);
            break;
        case des::RoomType::OTHER:
            nextExecutionTime = this->time + rnd::uni(ctx.rng(), 60, ONE_HOUR * 1);
            break;
    }

    if(p.departureTime < nextExecutionTime) {
        auto elevatorIt = std::find_if(p.roomLabels.begin(), p.roomLabels.end(),
            [](const std::string& r) { return r.find("Elevator") != std::string::npos; });
        if (elevatorIt != p.roomLabels.end()) {
            p.currentRoom = *elevatorIt;
        }
        ctx.pushEvent(std::make_shared<PersonDepartureEvent>(p.departureTime, this->person));
    } else {
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(nextExecutionTime, this->person));
    }
}

void PersonArrivedEvent::execute(ISimContext& ctx) {
    ctx.notifyEvent(*this);
    auto& p = *this->person;

    auto it = std::find(p.roomLabels.begin(), p.roomLabels.end(), p.currentRoom);

    // Person arrived at unknown room (e.g. after accompany) — notify, then return to workplace
    if (it == p.roomLabels.end()) {
        ctx.notifyEvent(*this);
        auto returnCopy = std::make_shared<des::Person>(p);
        returnCopy->currentRoom = p.workplace;
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(
            this->time + rnd::uni(ctx.rng(), 60, ONE_HOUR), returnCopy));
        return;
    }

    int currentIndex = std::distance(p.roomLabels.begin(), it);
    const std::vector<double>& row = p.transitionMatrix.at(currentIndex);
    int nextRoomIdx = rnd::discrete_dist(ctx.rng(), row);

    p.currentRoom = p.roomLabels.at(nextRoomIdx);
    double nextExecutionTime = this->time + rnd::uni(ctx.rng(), 10, 30);
    ctx.pushEvent(std::make_shared<PersonTransitionEvent>(nextExecutionTime, this->person));
}

void PersonDepartureEvent::execute(ISimContext& ctx) {
    this->person->currentRoom = "OUTDOOR";
    ctx.notifyEvent(*this);
}

void MissionStartEvent::execute(ISimContext& ctx) {
    const std::string person = appointment->personName;
    assert(ctx.hasEmployee(person));
    const auto& locations = ctx.getPersonByName(person)->roomLabels;
    assert(!locations.empty());
    ctx.changeRobotState(std::make_unique<SearchState>(locations));
    ctx.notifyEvent(*this);
    ctx.tickBT();
}
