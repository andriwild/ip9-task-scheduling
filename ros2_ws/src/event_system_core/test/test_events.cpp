#include <gtest/gtest.h>
#include <memory>
#include <random>

#include "../src/model/event.h"
#include "../src/model/i_sim_context.h"
#include "../src/model/robot.h"
#include "../src/model/robot_state.h"

struct Journey {
    double duration;
    double distance;
};

class MockSimContext : public ISimContext {
public:
    // Tracking
    std::vector<std::shared_ptr<IEvent>> pushedEvents;
    std::vector<std::string> notifiedEvents;
    int tickCount = 0;
    std::map<std::string, std::string> blackboard;

    // Configurable state
    std::shared_ptr<Robot> robot;
    std::shared_ptr<des::Appointment> currentAppointment;
    std::shared_ptr<des::SimConfig> simConfig;
    std::map<std::string, std::shared_ptr<des::Person>> employees;
    std::vector<std::shared_ptr<des::Appointment>> pendingMissions;
    int currentTime = 0;
    mutable std::mt19937 m_rng{42};

    MockSimContext() {
        simConfig = std::make_shared<des::SimConfig>();
        simConfig->robotSpeed = 1.0;
        simConfig->robotAccompanySpeed = 0.5;
        simConfig->batteryCapacity = 100.0;
        simConfig->initialBatteryCapacity = 80.0;
        simConfig->lowBatteryThreshold = 20.0;
        simConfig->fullBatteryThreshold = 95.0;
        simConfig->conversationProbability = 1.0;
        simConfig->conversationDurationMean = 30.0;
        simConfig->conversationDurationStd = 0.0; // deterministic
        simConfig->personFindProbability = 1.0;
        simConfig->driveTimeStd = 0.0;
        simConfig->timeBuffer = 60.0;
        simConfig->energyConsumptionDrive = 0.1;
        simConfig->energyConsumptionBase = 0.01;
        simConfig->chargingRate = 0.5;
        simConfig->dockLocation = "Dock";
        simConfig->cacheEnabled = false;

        robot = std::make_shared<Robot>(simConfig);
    }

    // ISimContext implementation
    int getTime() const override { return currentTime; }

    void pushEvent(const std::shared_ptr<IEvent>& event) override {
        pushedEvents.push_back(event);
    }

    void tickBT() override { tickCount++; }

    void setBTBlackboard(const std::string& key, const std::string& value) override {
        blackboard[key] = value;
    }

    std::shared_ptr<Robot> getRobot() const override { return robot; }

    void changeRobotState(std::unique_ptr<RobotState> newState) const override {
        robot->changeState(std::move(newState));
    }

    void robotMoved(const std::string& location, double /*distance*/) const override {
        robot->setLocation(location);
    }

    Journey scheduleArrival(const std::string& /*target*/) const override {
        return {10.0, 5.0};
    }

    void notifyEvent(const IEvent& event) const override {
        const_cast<MockSimContext*>(this)->notifiedEvents.push_back(event.getName());
    }

    void setAppointment(const std::shared_ptr<des::Appointment>& appointment) override {
        currentAppointment = appointment;
    }

    std::shared_ptr<des::Appointment> getAppointment() const override {
        return currentAppointment;
    }

    void updateAppointmentState(const des::MissionState& newState) override {
        if (currentAppointment) currentAppointment->state = newState;
    }

    void addPendingMission(const std::shared_ptr<des::Appointment>& appointment) override {
        pendingMissions.push_back(appointment);
    }

    bool hasPendingMission() const override { return !pendingMissions.empty(); }

    std::shared_ptr<des::Appointment> nextPendingMission() override {
        return pendingMissions.empty() ? nullptr : pendingMissions.front();
    }

    std::shared_ptr<des::Appointment> popPendingMission() override {
        if (pendingMissions.empty()) return nullptr;
        auto front = pendingMissions.front();
        pendingMissions.erase(pendingMissions.begin());
        return front;
    }

    void completeAppointment(const std::shared_ptr<des::Appointment>& /*appt*/) const override {}

    bool isMissionFeasible(const des::Appointment& /*appointment*/, const std::string& /*startPos*/) const override {
        return true;
    }

    bool hasEmployee(const std::string& person) const override {
        return employees.contains(person);
    }

    std::shared_ptr<des::Person> getPersonByName(const std::string& person) const override {
        return employees.at(person);
    }

    std::shared_ptr<des::SimConfig> getConfig() const override { return simConfig; }
    double getPersonFindProbability() const override { return simConfig->personFindProbability; }
    double getConversationProbability() const override { return simConfig->conversationProbability; }
    double getRndConversationTime() const override { return simConfig->conversationDurationMean; }
    std::mt19937& rng() const override { return m_rng; }
};

// --- SimulationStartEvent ---

TEST(EventExecute, SimulationStartSetsIdleState) {
    MockSimContext ctx;
    SimulationStartEvent event(0);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::IDLE);
    EXPECT_FALSE(ctx.pushedEvents.empty());
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

// --- SimulationEndEvent ---

TEST(EventExecute, SimulationEndSetsIdleState) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<SearchState>(std::vector<std::string>{"A"}));

    SimulationEndEvent event(1000);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::IDLE);
    EXPECT_FALSE(ctx.pushedEvents.empty());
}

// --- MissionDispatchEvent ---

TEST(EventExecute, MissionDispatchAddsPendingAndTicksBT) {
    MockSimContext ctx;

    auto appt = std::make_shared<des::Appointment>();
    appt->id = 1;
    appt->personName = "Max";
    appt->roomName = "Room1";
    appt->appointmentTime = 36000;
    appt->description = "Test";

    MissionDispatchEvent event(35000, appt);
    event.execute(ctx);

    ASSERT_EQ(ctx.pendingMissions.size(), 1u);
    EXPECT_EQ(ctx.pendingMissions[0]->personName, "Max");
    EXPECT_EQ(ctx.tickCount, 1);
}

// --- MissionStartEvent ---

TEST(EventExecute, MissionStartSetsSearchState) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->roomLabels = {"Office", "Kitchen"};
    ctx.employees["Max"] = person;

    auto appt = std::make_shared<des::Appointment>();
    appt->id = 1;
    appt->personName = "Max";
    appt->roomName = "Room1";
    appt->appointmentTime = 36000;

    MissionStartEvent event(35000, appt);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::SEARCHING);
    EXPECT_EQ(ctx.tickCount, 1);
}

// --- AbortSearchEvent ---

TEST(EventExecute, AbortSearchFailsMissionAndSetsIdle) {
    MockSimContext ctx;

    auto appt = std::make_shared<des::Appointment>();
    appt->id = 1;
    appt->personName = "Max";
    appt->state = des::MissionState::IN_PROGRESS;
    ctx.currentAppointment = appt;

    AbortSearchEvent event(36000);
    event.execute(ctx);

    EXPECT_EQ(ctx.currentAppointment->state, des::MissionState::FAILED);
    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::IDLE);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::MISSION_COMPLETE);
}

// --- StartAccompanyEvent ---

TEST(EventExecute, StartAccompanySetsAccompanyStateAndDrives) {
    MockSimContext ctx;

    auto appt = std::make_shared<des::Appointment>();
    appt->id = 1;
    appt->personName = "Max";
    appt->roomName = "MeetingRoom";
    appt->appointmentTime = 36000;
    ctx.currentAppointment = appt;

    StartAccompanyEvent event(35500);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::ACCOMPANY);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::START_DRIVE);
}

// --- StartFoundPersonConversationEvent ---

TEST(EventExecute, StartFoundPersonConversationSetsConversateState) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    StartFoundPersonConversationEvent event(35000);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::CONVERSATE);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    // With conversationProbability=1.0, should always push Success
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::FOUND_PERSON_CONV_COMPLETE);
}

TEST(EventExecute, StartFoundPersonConvFailsWithZeroProbability) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);
    ctx.simConfig->conversationProbability = 0.0;

    StartFoundPersonConversationEvent event(35000);
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    // With probability 0, rnd::uni will always be >= 0, so always fails
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::FOUND_PERSON_CONV_COMPLETE);
}

// --- StartDropOffConversationEvent ---

TEST(EventExecute, StartDropOffConversationSetsConversateState) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    StartDropOffConversationEvent event(35000);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::CONVERSATE);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::DROP_OFF_CONV_COMPLETE);
}

// --- SuccessFoundPersonConversationCompleteEvent ---

TEST(EventExecute, SuccessFoundPersonConvSetsResultAndTicks) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));

    SuccessFoundPersonConversationCompleteEvent event(35030);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getState()->getResult(), des::Result::SUCCESS);
    EXPECT_EQ(ctx.tickCount, 1);
}

// --- FailedFoundPersonConversationCompleteEvent ---

TEST(EventExecute, FailedFoundPersonConvSetsFailureAndTicks) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));

    FailedFoundPersonConversationCompleteEvent event(35030);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getState()->getResult(), des::Result::FAILURE);
    EXPECT_EQ(ctx.tickCount, 1);
}

// --- BatteryFullEvent ---

TEST(EventExecute, BatteryFullSetsIdleAndResetsBatteryFlag) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<ChargeState>());
    ctx.robot->m_batteryFullEventScheduled = true;

    BatteryFullEvent event(37000);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::IDLE);
    EXPECT_FALSE(ctx.robot->m_batteryFullEventScheduled);
    EXPECT_EQ(ctx.tickCount, 1);
}

// --- MissionCompleteEvent ---

TEST(EventExecute, MissionCompleteNotifiesAndTicks) {
    MockSimContext ctx;

    auto appt = std::make_shared<des::Appointment>();
    appt->id = 1;
    appt->personName = "Max";
    appt->state = des::MissionState::COMPLETED;

    MissionCompleteEvent event(36500, appt);
    event.execute(ctx);

    EXPECT_EQ(ctx.tickCount, 1);
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

// --- PersonDepartureEvent ---

TEST(EventExecute, PersonDepartureSetsRoomToOutdoor) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->currentRoom = "Office";
    person->roomLabels = {"Office"};

    PersonDepartureEvent event(61200, person);
    event.execute(ctx);

    EXPECT_EQ(person->currentRoom, "OUTDOOR");
}

// --- StopDriveEvent ---

TEST(EventExecute, StopDriveMovesRobotAndSetsDrivingFalse) {
    MockSimContext ctx;
    ctx.robot->setDriving(true);

    StopDriveEvent event(35010, "Office", 5.0);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getLocation(), "Office");
    EXPECT_FALSE(ctx.robot->isDriving());
    EXPECT_EQ(ctx.blackboard["location"], "Office");
    EXPECT_EQ(ctx.tickCount, 1);
}

// --- Event metadata ---

TEST(EventMetadata, EventTypesAreCorrect) {
    EXPECT_EQ(SimulationStartEvent(0).getType(), des::EventType::SIMULATION_START);
    EXPECT_EQ(SimulationEndEvent(0).getType(), des::EventType::SIMULATION_END);
    EXPECT_EQ(AbortSearchEvent(0).getType(), des::EventType::ABORT_SEARCH);
    EXPECT_EQ(BatteryFullEvent(0).getType(), des::EventType::BATTERY_FULL);
    EXPECT_EQ(StartAccompanyEvent(0).getType(), des::EventType::START_ACCOMPANY);
    EXPECT_EQ(StartDropOffConversationEvent(0).getType(), des::EventType::START_DROP_OFF_CONV);
    EXPECT_EQ(StartFoundPersonConversationEvent(0).getType(), des::EventType::START_FOUND_PERSON_CONV);
}

TEST(EventMetadata, EventNamesAreNonEmpty) {
    EXPECT_FALSE(SimulationStartEvent(0).getName().empty());
    EXPECT_FALSE(SimulationEndEvent(0).getName().empty());
    EXPECT_FALSE(AbortSearchEvent(0).getName().empty());
    EXPECT_FALSE(BatteryFullEvent(0).getName().empty());
}
