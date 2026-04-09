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
            const auto& arrivalLocation = ctx.getRobot()->getLocation();
            ctx.setPersonLocation(personName, arrivalLocation);
            auto transition = std::make_shared<PersonTransitionEvent>(this->time, person);
            transition->targetRoom = arrivalLocation;
            ctx.pushEvent(transition);
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
    ctx.getRobot()->setIsPersonVisible(false);
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
    const auto& personName = ctx.getAppointment()->personName;
    if (ctx.hasEmployee(personName)) {
        ctx.getPersonByName(personName)->busy = true;
    }
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
    const auto& personName = this->appointment->personName;
    if (ctx.hasEmployee(personName)) {
        auto person = ctx.getPersonByName(personName);
        int endTime = this->time + static_cast<int>(ctx.getConfig()->appointmentDuration);
        ctx.pushEvent(std::make_shared<AppointmentEndEvent>(endTime, person));
    }
    ctx.completeAppointment(this->appointment);
    ctx.notifyEvent(*this);
    ctx.tickBT();
}

void AppointmentEndEvent::execute(ISimContext& ctx) {
    person->busy = false;
    ctx.pushEvent(std::make_shared<PersonTransitionEvent>(this->time, person));
    ctx.notifyEvent(*this);
}

void BatteryFullEvent::execute(ISimContext& ctx) {
    ctx.changeRobotState(std::make_unique<IdleState>());
    ctx.getRobot()->m_batteryFullEventScheduled = false;
    ctx.notifyEvent(*this);
    ctx.tickBT();
}

void PersonTransitionEvent::execute(ISimContext& ctx) {
    auto& p = *this->person;

    if (p.busy) {
        if (!targetRoom.empty()) {
            ctx.notifyEvent(*this);
        }
        return;
    }

    const std::string currentRoom = ctx.getPersonLocation(p.firstName);
    targetRoom = currentRoom;

    if (currentRoom == "OUTDOOR") {
        ctx.notifyEvent(*this);
        return;
    }

    auto it = std::find(p.roomLabels.begin(), p.roomLabels.end(), currentRoom);

    // Person was moved outside their known rooms (e.g. by accompany) — return to workplace
    if (it == p.roomLabels.end()) {
        ctx.notifyEvent(*this);
        ctx.setPersonLocation(p.firstName, p.workplace);
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(
            this->time + rnd::uni(ctx.rng(), 60, ONE_HOUR), this->person));
        return;
    }

    int currentIndex = std::distance(p.roomLabels.begin(), it);
    const std::vector<double>& row = p.transitionMatrix.at(currentIndex);
    int nextRoomIdx = rnd::discrete_dist(ctx.rng(), row);

    const std::string nextRoom = p.roomLabels.at(nextRoomIdx);
    targetRoom = nextRoom;
    ctx.notifyEvent(*this);
    ctx.setPersonLocation(p.firstName, nextRoom);

    double nextExecutionTime = this->time + p.getStayDuration(nextRoom, ctx.rng());

    if(p.departureTime < nextExecutionTime) {
        auto elevatorIt = std::find_if(p.roomLabels.begin(), p.roomLabels.end(),
            [](const std::string& r) { return r.find("Elevator") != std::string::npos; });
        if (elevatorIt != p.roomLabels.end()) {
            ctx.setPersonLocation(p.firstName, *elevatorIt);
        }
        ctx.pushEvent(std::make_shared<PersonDepartureEvent>(p.departureTime, this->person));
    } else {
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(nextExecutionTime, this->person));
    }
}

void PersonArrivedEvent::execute(ISimContext& ctx) {
    auto& p = *this->person;

    if (p.busy) {
        return;
    }

    const std::string currentRoom = ctx.getPersonLocation(p.firstName);
    targetRoom = currentRoom;
    ctx.notifyEvent(*this);

    auto it = std::find(p.roomLabels.begin(), p.roomLabels.end(), currentRoom);

    // Person arrived at unknown room (e.g. after accompany) — return to workplace
    if (it == p.roomLabels.end()) {
        ctx.setPersonLocation(p.firstName, p.workplace);
        ctx.pushEvent(std::make_shared<PersonTransitionEvent>(
            this->time + rnd::uni(ctx.rng(), 60, ONE_HOUR), this->person));
        return;
    }

    int currentIndex = std::distance(p.roomLabels.begin(), it);
    const std::vector<double>& row = p.transitionMatrix.at(currentIndex);
    int nextRoomIdx = rnd::discrete_dist(ctx.rng(), row);

    ctx.setPersonLocation(p.firstName, p.roomLabels.at(nextRoomIdx));
    double nextExecutionTime = this->time + rnd::uni(ctx.rng(), 10, 30);
    ctx.pushEvent(std::make_shared<PersonTransitionEvent>(nextExecutionTime, this->person));
}

void PersonDepartureEvent::execute(ISimContext& ctx) {
    if (this->person->busy) {
        return;
    }
    targetRoom = "OUTDOOR";
    ctx.notifyEvent(*this);
    ctx.setPersonLocation(this->person->firstName, "OUTDOOR");
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

void ScanAera::execute(ISimContext& ctx) {
    ctx.notifyEvent(*this);
    ctx.getRobot()->setScanning(true);

    const auto& personName = ctx.getAppointment()->personName;
    const std::string personLocation = ctx.getPersonLocation(personName);
    const std::string robotLocation  = ctx.getRobot()->getLocation();
    const bool personPresent = robotLocation == personLocation;
    const double areaToSearch = ctx.getSearchArea(robotLocation);
    const double fieldOfView  = ctx.getConfig()->personDetectionRange * ctx.getConfig()->personDetectionRange;
    assert(fieldOfView != 0);
    const double steps = areaToSearch / fieldOfView;
    const int scanTime = steps * (2 *  ctx.getConfig()->personDetectionRange / ctx.getConfig()->robotSpeed);
    assert(scanTime != 0);
    const int foundAt = rnd::uni(ctx.rng(), 1, scanTime);

    if (personPresent) {
        ctx.pushEvent(std::make_shared<ScanComplete>(this->time + foundAt, personPresent, scanTime - foundAt));
    } else {
        ctx.pushEvent(std::make_shared<ScanComplete>(this->time + scanTime, personPresent, 0));
    }
}

void ScanComplete::execute(ISimContext & ctx) {
    const auto& personName = ctx.getAppointment()->personName;
    const std::string personLocation = ctx.getPersonLocation(personName);
    const std::string robotLocation  = ctx.getRobot()->getLocation();
    const bool personPresent = robotLocation == personLocation;
    if (personPresent) {
        ctx.getRobot()->setIsPersonVisible(true);
    } else if(this->m_remainigSearchTime > 0) {
        ctx.pushEvent(std::make_shared<ScanComplete>(this->time + this->m_remainigSearchTime, personPresent, 0));
        return;
    }
    ctx.notifyEvent(*this);
    ctx.tickBT();
}
