#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <stdexcept>

#include "../src/model/context.h"
#include "../src/model/event.h"
#include "../src/model/i_sim_context.h"
#include "../src/model/robot.h"
#include "../src/model/robot_state.h"
#include "../src/plugins/accompany/accompany_order.h"
#include "../src/plugins/accompany/events/abort_search_event.h"
#include "../src/plugins/accompany/events/start_accompany_event.h"
#include "../src/plugins/accompany/events/start_found_person_conversation_event.h"
#include "../src/plugins/accompany/events/start_drop_off_conversation_event.h"
#include "../src/plugins/accompany/events/found_person_conversation_complete.h"
#include "../src/plugins/accompany/events/drop_off_conversation_complete.h"

class MockSimContext : public ISimContext {
public:
    // Tracking
    EventList pushedEvents;
    std::vector<std::string> notifiedEvents;
    int tickCount = 0;
    bool completeOrderCalled = false;
    std::map<std::string, std::string> blackboard;

    // Configurable state
    std::shared_ptr<Robot> robot;
    des::OrderPtr currentOrder;
    std::shared_ptr<des::SimConfig> simConfig;
    des::PersonLocationMap employees;
    std::map<std::string, std::string> personLocations;
    des::OrderList pendingMissions;
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
        simConfig->conversationDurationStd = 0.0;
        simConfig->driveTimeStd = 0.0;
        simConfig->timeBuffer = 60.0;
        simConfig->energyConsumptionDrive = 0.1;
        simConfig->energyConsumptionBase = 0.01;
        simConfig->chargingRate = 0.5;
        simConfig->dockLocation = "Dock";
        simConfig->cacheEnabled = false;

        robot = std::make_shared<Robot>(simConfig);
    }

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

    const Scheduler& getScheduler() const override {
        throw std::runtime_error("MockSimContext::getScheduler not implemented");
    }

    void notifyEvent(const IEvent& event) const override {
        const_cast<MockSimContext*>(this)->notifiedEvents.push_back(event.getName());
    }

    void setOrderPtr(const des::OrderPtr& order) override {
        currentOrder = order;
    }

    des::OrderPtr getOrderPtr() const override {
        return currentOrder;
    }

    void updateOrderState(const des::MissionState& newState) override {
        if (currentOrder) currentOrder->state = newState;
    }

    void addPendingMission(const des::OrderPtr order) override {
        pendingMissions.push_back(order);
    }

    bool hasPendingMission() const override { return !pendingMissions.empty(); }

    des::OrderPtr nextPendingMission() override {
        return pendingMissions.empty() ? nullptr : pendingMissions.front();
    }

    des::OrderPtr popPendingMission() override {
        if (pendingMissions.empty()) return nullptr;
        auto front = pendingMissions.front();
        pendingMissions.erase(pendingMissions.begin());
        return front;
    }

    void completeOrder(const des::OrderPtr& /*order*/) override {
        completeOrderCalled = true;
    }

    bool hasEmployee(const std::string& person) const override {
        return employees.contains(person);
    }

    std::shared_ptr<des::Person> getPersonByName(const std::string& person) const override {
        return employees.at(person);
    }

    std::shared_ptr<des::SimConfig> getConfig() const override { return simConfig; }
    double getConversationProbability() const override { return simConfig->conversationProbability; }
    double getRndConversationTime() const override { return simConfig->conversationDurationMean; }
    std::mt19937& rng() const override { return m_rng; }

    std::string getPersonLocation(const std::string& name) const override {
        return personLocations.at(name);
    }
    void setPersonLocation(const std::string& name, const std::string& room) override {
        personLocations[name] = room;
    }
    double getSearchArea(const std::string& /*name*/) const override { return 0.0; }
};

static std::shared_ptr<AccompanyOrder> makeAccompanyOrder(
        int id,
        const std::string& person,
        const std::string& room = "Room1",
        int appointmentTime = 36000,
        const std::string& description = "Test") {
    auto o = std::make_shared<AccompanyOrder>();
    o->id = id;
    o->type = "accompany";
    o->personName = person;
    o->roomName = room;
    o->deadline = appointmentTime;
    o->description = description;
    return o;
}

// --- SimulationStartEvent ---

TEST(EventExecute, SimulationStartSetsIdleState) {
    MockSimContext ctx;
    SimulationStartEvent event(0);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::IDLE);
    EXPECT_FALSE(ctx.pushedEvents.empty());
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

TEST(EventExecute, SimulationStartPushesStopDriveEvent) {
    MockSimContext ctx;
    SimulationStartEvent event(500);
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx.pushedEvents[0]->time, 500);
}

// --- SimulationEndEvent ---

TEST(EventExecute, SimulationEndSetsIdleState) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<SearchState>(std::vector<std::string>{"A"}));

    SimulationEndEvent event(1000);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::IDLE);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::STOP_DRIVE);
}

// --- MissionDispatchEvent ---

TEST(EventExecute, MissionDispatchAddsPendingAndTicksBT) {
    MockSimContext ctx;

    auto order = makeAccompanyOrder(1, "Max");

    MissionDispatchEvent event(35000, order);
    event.execute(ctx);

    ASSERT_EQ(ctx.pendingMissions.size(), 1u);
    auto accompany = std::dynamic_pointer_cast<AccompanyOrder>(ctx.pendingMissions[0]);
    ASSERT_NE(accompany, nullptr);
    EXPECT_EQ(accompany->personName, "Max");
    EXPECT_EQ(ctx.tickCount, 1);
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

// --- MissionStartEvent ---

TEST(EventExecute, MissionStartSetsSearchState) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->roomLabels = {"Office", "Kitchen"};
    ctx.employees["Max"] = person;

    auto order = makeAccompanyOrder(1, "Max");

    MissionStartEvent event(35000, order);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::SEARCHING);
    EXPECT_EQ(ctx.tickCount, 1);

    // Verify SearchState has the correct locations
    auto* searchState = dynamic_cast<SearchState*>(ctx.robot->getState());
    ASSERT_NE(searchState, nullptr);
    ASSERT_EQ(searchState->locations.size(), 2u);
    EXPECT_EQ(searchState->locations[0], "Office");
    EXPECT_EQ(searchState->locations[1], "Kitchen");
}

// --- AbortSearchEvent ---

TEST(EventExecute, AbortSearchFailsMissionAndSetsIdle) {
    MockSimContext ctx;

    auto order = makeAccompanyOrder(1, "Max");
    order->state = des::MissionState::IN_PROGRESS;
    ctx.currentOrder = order;

    AbortSearchEvent event(36000);
    event.execute(ctx);

    EXPECT_EQ(ctx.currentOrder->state, des::MissionState::FAILED);
    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::IDLE);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::MISSION_COMPLETE);
}

// --- StartAccompanyEvent ---

TEST(EventExecute, StartAccompanySetsAccompanyStateAndDrivesToRoom) {
    MockSimContext ctx;

    auto order = makeAccompanyOrder(1, "Max", "MeetingRoom");
    ctx.currentOrder = order;

    StartAccompanyEvent event(35500);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::ACCOMPANY);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::START_DRIVE);
    // Verify the drive target is the appointment room
    auto* driveEvent = dynamic_cast<StartDriveEvent*>(ctx.pushedEvents[0].get());
    ASSERT_NE(driveEvent, nullptr);
    EXPECT_EQ(driveEvent->getName(), "Departing: MeetingRoom");
}

// --- StartFoundPersonConversationEvent ---

TEST(EventExecute, StartFoundPersonConvPushesSuccessWithHighProbability) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);
    ctx.simConfig->conversationProbability = 1.0;

    StartFoundPersonConversationEvent event(35000);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::CONVERSATE);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    // Verify it's the Success variant by checking getName()
    EXPECT_EQ(ctx.pushedEvents[0]->getName(), "Conversation Successful");
}

TEST(EventExecute, StartFoundPersonConvPushesFailedWithZeroProbability) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);
    ctx.simConfig->conversationProbability = 0.0;

    StartFoundPersonConversationEvent event(35000);
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    // With probability=0, rnd::uni >= 0 always, so Failed variant must be pushed
    EXPECT_EQ(ctx.pushedEvents[0]->getName(), "Conversation Failed ");
}

TEST(EventExecute, StartFoundPersonConvEventTimeIncludesConversationDuration) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);
    ctx.simConfig->conversationDurationMean = 45.0;

    StartFoundPersonConversationEvent event(35000);
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->time, 35000 + 45);
}

// --- StartDropOffConversationEvent ---

TEST(EventExecute, StartDropOffConvPushesSuccessWithHighProbability) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);
    ctx.simConfig->conversationProbability = 1.0;

    StartDropOffConversationEvent event(35000);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::CONVERSATE);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getName(), "Conversation Successful");
}

TEST(EventExecute, StartDropOffConvPushesFailedWithZeroProbability) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);
    ctx.simConfig->conversationProbability = 0.0;

    StartDropOffConversationEvent event(35000);
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getName(), "Conversation Failed ");
}

// --- Conversation complete events ---

TEST(EventExecute, SuccessFoundPersonConvSetsResultAndTicks) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));

    SuccessFoundPersonConversationCompleteEvent event(35030);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getState()->getResult(), des::Result::SUCCESS);
    EXPECT_EQ(ctx.tickCount, 1);
}

TEST(EventExecute, FailedFoundPersonConvSetsFailureAndTicks) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));

    FailedFoundPersonConversationCompleteEvent event(35030);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getState()->getResult(), des::Result::FAILURE);
    EXPECT_EQ(ctx.tickCount, 1);
}

TEST(EventExecute, SuccessDropOffConvSetsResultAndTicks) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<ConversateState>(ConversateState::Type::DROP_OFF));

    SuccessDropOffConversationCompleteEvent event(35030);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getState()->getResult(), des::Result::SUCCESS);
    EXPECT_EQ(ctx.tickCount, 1);
}

TEST(EventExecute, FailedDropOffConvSetsFailureAndTicks) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<ConversateState>(ConversateState::Type::DROP_OFF));

    FailedDropOffConversationCompleteEvent event(35030);
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

TEST(EventExecute, MissionCompleteCallsCompleteOrderAndTicks) {
    MockSimContext ctx;

    auto order = makeAccompanyOrder(1, "Max");
    order->state = des::MissionState::COMPLETED;

    MissionCompleteEvent event(36500, order);
    event.execute(ctx);

    EXPECT_TRUE(ctx.completeOrderCalled);
    EXPECT_EQ(ctx.tickCount, 1);
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

// --- StartDriveEvent ---

TEST(EventExecute, StartDriveSetsDrivingAndSchedulesArrival) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    StartDriveEvent event(35000, "Office");
    event.execute(ctx);

    EXPECT_TRUE(ctx.robot->isDriving());
    EXPECT_EQ(ctx.robot->getTargetLocation(), "Office");
    // Should push a StopDriveEvent
    ASSERT_GE(ctx.pushedEvents.size(), 1u);
    bool hasStopDrive = false;
    for (const auto& e : ctx.pushedEvents) {
        if (e->getType() == des::EventType::STOP_DRIVE) {
            hasStopDrive = true;
        }
    }
    EXPECT_TRUE(hasStopDrive);
}

TEST(EventExecute, StartDriveToSameLocationPushesImmediateStop) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);
    ctx.robot->setLocation("Office");

    StartDriveEvent event(35000, "Office");
    event.execute(ctx);

    // When already at location, should push StopDriveEvent with time=35000
    bool hasImmediateStop = false;
    for (const auto& e : ctx.pushedEvents) {
        if (e->getType() == des::EventType::STOP_DRIVE && e->time == 35000) {
            hasImmediateStop = true;
        }
    }
    EXPECT_TRUE(hasImmediateStop);
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

TEST(EventExecute, StopDriveInAccompanyMovesPerson) {
    MockSimContext ctx;
    ctx.robot->setDriving(true);
    ctx.robot->changeState(std::make_unique<AccompanyState>());

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->roomLabels = {"Office", "MeetingRoom"};
    ctx.employees["Max"] = person;
    ctx.personLocations["Max"] = "Office";

    auto order = makeAccompanyOrder(1, "Max", "MeetingRoom");
    ctx.currentOrder = order;

    StopDriveEvent event(35100, "MeetingRoom", 10.0);
    event.execute(ctx);

    // Person should be moved to the robot's arrival location
    EXPECT_EQ(ctx.personLocations["Max"], "MeetingRoom");
}

// --- PersonDepartureEvent ---

TEST(EventExecute, PersonDepartureSetsRoomToOutdoor) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->roomLabels = {"Office"};
    ctx.personLocations["Max"] = "Office";

    PersonDepartureEvent event(61200, person);
    event.execute(ctx);

    EXPECT_EQ(ctx.personLocations["Max"], "OUTDOOR");
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

// --- PersonTransitionEvent ---

TEST(EventExecute, PersonTransitionFromOutdoorDoesNotPushEvent) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->roomLabels = {"Office", "Kitchen"};
    ctx.personLocations["Max"] = "OUTDOOR";

    PersonTransitionEvent event(30000, person);
    event.execute(ctx);

    // Should only notify, not push any follow-up event
    EXPECT_TRUE(ctx.pushedEvents.empty());
    EXPECT_FALSE(ctx.notifiedEvents.empty());
    EXPECT_EQ(ctx.personLocations["Max"], "OUTDOOR");
}

TEST(EventExecute, PersonTransitionFromUnknownRoomReturnsToWorkplace) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->workplace = "Office";
    person->roomLabels = {"Office", "Kitchen"};
    ctx.personLocations["Max"] = "UnknownRoom";

    PersonTransitionEvent event(30000, person);
    event.execute(ctx);

    EXPECT_EQ(ctx.personLocations["Max"], "Office");
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_TRANSITION);
    // Follow-up time should be between 60 and 3600 seconds later
    EXPECT_GT(ctx.pushedEvents[0]->time, 30000);
    EXPECT_LE(ctx.pushedEvents[0]->time, 30000 + ONE_HOUR);
}

TEST(EventExecute, PersonTransitionMovesToNewRoomAndSchedulesNext) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->workplace = "5.2B03";
    person->departureTime = 999999; // far in the future
    person->roomLabels = {"5.2B03", "IMVS_Kitchen"};
    person->transitionMatrix = {
        {0.0, 1.0}, // from 5.2B03 always go to Kitchen
        {1.0, 0.0},
    };
    ctx.personLocations["Max"] = "5.2B03";

    PersonTransitionEvent event(30000, person);
    event.execute(ctx);

    // Person should have moved to Kitchen (100% probability)
    EXPECT_EQ(ctx.personLocations["Max"], "IMVS_Kitchen");
    // Should schedule a follow-up transition
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_TRANSITION);
    EXPECT_GT(ctx.pushedEvents[0]->time, 30000);
}

TEST(EventExecute, PersonTransitionSchedulesDepartureWhenTimeExceeded) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->workplace = "5.2B03";
    person->departureTime = 30001; // almost immediately
    person->roomLabels = {"5.2B03", "IMVS_Kitchen", "5.2B_Elevator"};
    person->transitionMatrix = {
        {0.0, 1.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
    };
    ctx.personLocations["Max"] = "5.2B03";

    PersonTransitionEvent event(30000, person);
    event.execute(ctx);

    // nextExecutionTime will be > departureTime, so should schedule departure
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_DEPARTURE);
    EXPECT_EQ(ctx.pushedEvents[0]->time, 30001);
    // Person should be moved to elevator
    EXPECT_EQ(ctx.personLocations["Max"], "5.2B_Elevator");
}

// --- PersonArrivedEvent ---

TEST(EventExecute, PersonArrivedSchedulesTransition) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->workplace = "5.2B03";
    person->roomLabels = {"5.2B03", "IMVS_Kitchen"};
    person->transitionMatrix = {
        {0.0, 1.0},
        {1.0, 0.0},
    };
    ctx.personLocations["Max"] = "5.2B03";

    PersonArrivedEvent event(30000, person);
    event.execute(ctx);

    // Should transition to a new room via the matrix
    EXPECT_EQ(ctx.personLocations["Max"], "IMVS_Kitchen");
    // Should push a PersonTransitionEvent with short delay (10-30s)
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_TRANSITION);
    EXPECT_GE(ctx.pushedEvents[0]->time, 30010);
    EXPECT_LE(ctx.pushedEvents[0]->time, 30030);
}

TEST(EventExecute, PersonArrivedAtUnknownRoomReturnsToWorkplace) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->workplace = "Office";
    person->roomLabels = {"Office", "Kitchen"};
    ctx.personLocations["Max"] = "UnknownRoom";

    PersonArrivedEvent event(30000, person);
    event.execute(ctx);

    EXPECT_EQ(ctx.personLocations["Max"], "Office");
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_TRANSITION);
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
    EXPECT_EQ(StopDriveEvent(0, "x", 0).getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(StartDriveEvent(0, "x").getType(), des::EventType::START_DRIVE);
}

TEST(EventMetadata, EventNamesAreNonEmpty) {
    EXPECT_FALSE(SimulationStartEvent(0).getName().empty());
    EXPECT_FALSE(SimulationEndEvent(0).getName().empty());
    EXPECT_FALSE(AbortSearchEvent(0).getName().empty());
    EXPECT_FALSE(BatteryFullEvent(0).getName().empty());
    EXPECT_FALSE(StopDriveEvent(0, "X", 0).getName().empty());
    EXPECT_FALSE(StartDriveEvent(0, "X").getName().empty());
}
