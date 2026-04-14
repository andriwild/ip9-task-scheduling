#include <gtest/gtest.h>
#include <memory>

#include "../src/behaviour/bt_setup.h"
#include "../src/init/config_loader.h"
#include "../src/model/context.h"
#include "../src/model/event.h"
#include "../src/model/event_queue.h"
#include "../src/observer/observer.h"
#include "../src/sim/i_path_planner.h"

class MockPathPlanner : public IPathPlanner {
    std::map<std::pair<std::string, std::string>, double> m_distances;

public:
    void setDistance(const std::string& from, const std::string& to, double dist) {
        m_distances[{from, to}] = dist;
        m_distances[{to, from}] = dist;
    }

    std::optional<double> calcDistance(const std::string& from, const std::string& to, bool) override {
        if (from == to) return 0.0;
        auto it = m_distances.find({from, to});
        return it != m_distances.end() ? std::optional(it->second) : std::nullopt;
    }
};

class TrackingObserver : public IObserver {
public:
    std::vector<std::pair<int, des::EventType>> events;
    std::vector<std::pair<int, des::RobotStateType>> stateChanges;
    std::vector<std::pair<int, des::MissionState>> missionCompletions;
    std::vector<std::pair<int, std::string>> moves;

    std::string getName() override { return "TrackingObserver"; }

    void onEvent(int time, des::EventType type, const std::string&, bool, bool, const std::string&) override {
        events.emplace_back(time, type);
    }

    void onStateChanged(int time, const des::RobotStateType& type, des::BatteryProps) override {
        stateChanges.emplace_back(time, type);
    }

    void onMissionComplete(int time, const des::MissionState& state, int) override {
        missionCompletions.emplace_back(time, state);
    }

    void onRobotMoved(int time, const std::string& location, double) override {
        moves.emplace_back(time, location);
    }
};

class IntegrationTest : public ::testing::Test {
protected:
    EventQueue eventQueue;
    std::shared_ptr<MockPathPlanner> planner;
    std::shared_ptr<des::SimConfig> config;
    des::PersonLocationMap employeeLocations;
    des::SearchAreaMap searchAreas;
    std::shared_ptr<TrackingObserver> observer;

    void SetUp() override {
        planner = std::make_shared<MockPathPlanner>();
        observer = std::make_shared<TrackingObserver>();

        config = std::make_shared<des::SimConfig>();
        config->robotSpeed = 1.0;
        config->robotAccompanySpeed = 0.5;
        config->timeBuffer = 60.0;
        config->cacheEnabled = false;
        config->conversationProbability = 1.0;
        config->conversationDurationMean = 30.0;
        config->conversationDurationStd = 0.0;
        config->driveTimeStd = 0.0;
        config->energyConsumptionDrive = 0.1;
        config->energyConsumptionBase = 0.01;
        config->batteryCapacity = 100.0;
        config->initialBatteryCapacity = 80.0;
        config->chargingRate = 0.5;
        config->lowBatteryThreshold = 20.0;
        config->fullBatteryThreshold = 95.0;
        config->dockLocation = "IMVS_Dock";

        // Locations with distances
        planner->setDistance("IMVS_Dock", "Office", 10.0);
        planner->setDistance("Office", "MeetingRoom", 20.0);
        planner->setDistance("IMVS_Dock", "MeetingRoom", 25.0);

        // Employee
        auto max = std::make_shared<des::Person>();
        max->firstName = "Max";
        max->lastName = "Mustermann";
        max->workplace = "Office";
        max->roomLabels = {"Office"};
        max->transitionMatrix = {{1.0}};
        max->arrivalTime = 28800;
        max->departureTime = 61200;
        employeeLocations["Max"] = max;
    }

    // Run the event loop similar to main.cpp
    void runEventLoop(SimulationContext& ctx, int maxEvents = 200) {
        int processed = 0;
        while (!eventQueue.empty() && processed < maxEvents) {
            auto e = eventQueue.top();
            eventQueue.pop();
            ctx.advanceTime(e->time);
            e->execute(ctx);
            processed++;
        }
    }

    // Execute the next event from the queue and return it
    std::shared_ptr<IEvent> step(SimulationContext& ctx) {
        if (eventQueue.empty()) return nullptr;
        auto e = eventQueue.top();
        eventQueue.pop();
        ctx.advanceTime(e->time);
        e->execute(ctx);
        return e;
    }

};

// --- Config Roundtrip (kept here for completeness with integration) ---

TEST(ConfigRoundtrip, SaveAndReloadPreservesAllFields) {
    auto original = ConfigLoader::loadSimConfig(TEST_FIXTURES_DIR + std::string("/test_sim_config.json"));
    ASSERT_TRUE(original.has_value());

    std::string tmpFile = "/tmp/test_roundtrip_integration.json";
    auto ptr = std::make_shared<des::SimConfig>(*original);
    ASSERT_TRUE(ConfigLoader::saveSimConfig(tmpFile, ptr));

    auto reloaded = ConfigLoader::loadSimConfig(tmpFile);
    ASSERT_TRUE(reloaded.has_value());

    // Verify all distribution-related fields survive the roundtrip
    EXPECT_EQ(des::distributionTypeToString(original->arrivalDistribution),
              des::distributionTypeToString(reloaded->arrivalDistribution));
    EXPECT_EQ(des::distributionTypeToString(original->departureDistribution),
              des::distributionTypeToString(reloaded->departureDistribution));

    std::filesystem::remove(tmpFile);
}

// --- Event Loop: full scenario ---

TEST_F(IntegrationTest, SingleMissionCompletesSuccessfully) {
    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, searchAreas
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(setupBehaviorTree(ctx));

    // Create a simple scenario: one appointment
    auto appt = std::make_shared<des::Appointment>();
    appt->id = 0;
    appt->personName = "Max";
    appt->roomName = "MeetingRoom";
    appt->appointmentTime = 36000;
    appt->description = "Test Meeting";

    // Schedule mission (same as IAppRunner::createMissionQueue)
    des::AppointmentList appointments = {appt};
    auto missions = ctx->getScheduler().simplePlan(appointments, "IMVS_Dock");
    for (auto& m : missions) {
        m->time = m->time - config->timeBuffer;
        eventQueue.push(m);
    }

    // Add simulation start/end
    int startTime = eventQueue.getFirstEventTime() - ONE_HOUR;
    int endTime = 40000;
    eventQueue.push(std::make_shared<SimulationStartEvent>(startTime));
    eventQueue.push(std::make_shared<SimulationEndEvent>(endTime));

    ctx->resetContext(startTime);
    ctx->setPersonLocation("Max", "Office");
    runEventLoop(*ctx);

    // Verify: mission was completed
    EXPECT_FALSE(observer->missionCompletions.empty());
    // Verify: robot went through state changes
    EXPECT_FALSE(observer->stateChanges.empty());
    // Verify: robot moved at least once
    EXPECT_FALSE(observer->moves.empty());
    // Verify: multiple events were processed
    EXPECT_GT(observer->events.size(), 5u);

    // Verify: state changes include SEARCHING and ACCOMPANY phases
    bool hasSearch = false, hasAccompany = false, hasConversate = false;
    for (const auto& [time, state] : observer->stateChanges) {
        if (state == des::RobotStateType::SEARCHING) hasSearch = true;
        if (state == des::RobotStateType::ACCOMPANY) hasAccompany = true;
        if (state == des::RobotStateType::CONVERSATE) hasConversate = true;
    }
    EXPECT_TRUE(hasSearch);
    EXPECT_TRUE(hasAccompany);
    EXPECT_TRUE(hasConversate);
}

TEST_F(IntegrationTest, EventLoopDrainsQueue) {
    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, searchAreas
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(setupBehaviorTree(ctx));

    // Minimal scenario: just start and end
    eventQueue.push(std::make_shared<SimulationStartEvent>(1000));
    eventQueue.push(std::make_shared<SimulationEndEvent>(2000));
    ctx->resetContext(1000);

    runEventLoop(*ctx);

    // Queue should be empty after processing
    EXPECT_TRUE(eventQueue.empty());
}

TEST_F(IntegrationTest, MissionDispatchWithoutPriorStartIsPending) {
    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, searchAreas
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(setupBehaviorTree(ctx));

    auto appt = std::make_shared<des::Appointment>();
    appt->id = 0;
    appt->personName = "Max";
    appt->roomName = "MeetingRoom";
    appt->appointmentTime = 36000;
    appt->description = "Test";

    // Start simulation, then dispatch mission
    eventQueue.push(std::make_shared<SimulationStartEvent>(30000));
    eventQueue.push(std::make_shared<MissionDispatchEvent>(34000, appt));
    eventQueue.push(std::make_shared<SimulationEndEvent>(40000));
    ctx->resetContext(30000);
    ctx->setPersonLocation("Max", "Office");

    runEventLoop(*ctx);

    // The mission should have been processed through the BT
    // After dispatch, BT ticks and accepts/rejects based on feasibility
    bool hasMissionDispatch = false;
    for (const auto& [time, type] : observer->events) {
        if (type == des::EventType::MISSION_DISPATCH) hasMissionDispatch = true;
    }
    EXPECT_TRUE(hasMissionDispatch);
}

// --- Reset behavior ---

TEST_F(IntegrationTest, ResetContextClearsStateAndResetsRobot) {
    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, searchAreas
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(setupBehaviorTree(ctx));

    // Run some events to change state
    eventQueue.push(std::make_shared<SimulationStartEvent>(1000));
    auto appt = std::make_shared<des::Appointment>();
    appt->id = 0;
    appt->personName = "Max";
    appt->roomName = "MeetingRoom";
    appt->appointmentTime = 5000;
    appt->description = "Test";
    eventQueue.push(std::make_shared<MissionDispatchEvent>(2000, appt));
    ctx->resetContext(1000);

    // Process start event
    auto e = eventQueue.top();
    eventQueue.pop();
    ctx->advanceTime(e->time);
    e->execute(*ctx);

    EXPECT_EQ(ctx->getTime(), 1000);

    // Reset
    ctx->resetContext(5000);

    EXPECT_EQ(ctx->getTime(), 5000);
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), config->dockLocation);
    EXPECT_FALSE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getAppointment(), nullptr);
    EXPECT_FALSE(ctx->hasPendingMission());
}

TEST_F(IntegrationTest, ResetContextAllowsRerun) {
    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, searchAreas
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(setupBehaviorTree(ctx));

    // First run
    eventQueue.push(std::make_shared<SimulationStartEvent>(1000));
    eventQueue.push(std::make_shared<SimulationEndEvent>(2000));
    ctx->resetContext(1000);
    runEventLoop(*ctx);

    size_t firstRunEvents = observer->events.size();
    EXPECT_GT(firstRunEvents, 0u);

    // Reset and run again
    ctx->resetContext(3000);
    eventQueue.push(std::make_shared<SimulationStartEvent>(3000));
    eventQueue.push(std::make_shared<SimulationEndEvent>(4000));
    runEventLoop(*ctx);

    // Should have processed more events
    EXPECT_GT(observer->events.size(), firstRunEvents);
}

// --- Observer integration ---

TEST_F(IntegrationTest, ObserverReceivesEventsInOrder) {
    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, searchAreas
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(setupBehaviorTree(ctx));

    eventQueue.push(std::make_shared<SimulationStartEvent>(1000));
    eventQueue.push(std::make_shared<SimulationEndEvent>(5000));
    ctx->resetContext(1000);

    runEventLoop(*ctx);

    // Events should be in chronological order
    for (size_t i = 1; i < observer->events.size(); ++i) {
        EXPECT_GE(observer->events[i].first, observer->events[i - 1].first)
            << "Event at index " << i << " is out of order";
    }
}

// --- Step-by-step event execution ---
//
// Traces a complete single-mission scenario event by event.
// After each step the robot state, location, driving flag, and
// mission state are verified so that regressions in the event
// chain are caught precisely.

TEST_F(IntegrationTest, StepByStepSingleMission) {
    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, searchAreas
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(setupBehaviorTree(ctx));

    // --- Setup: one appointment for Max at MeetingRoom @ 36000 (10:00) ---
    auto appt = std::make_shared<des::Appointment>();
    appt->id = 0;
    appt->personName = "Max";
    appt->roomName = "MeetingRoom";
    appt->appointmentTime = 36000;
    appt->description = "Dokument abholen";

    des::AppointmentList appointments = {appt};
    auto missions = ctx->getScheduler().simplePlan(appointments, "IMVS_Dock");
    for (auto& m : missions) {
        m->time = m->time - config->timeBuffer;
        eventQueue.push(m);
    }

    int startTime = eventQueue.getFirstEventTime() - ONE_HOUR;
    eventQueue.push(std::make_shared<SimulationStartEvent>(startTime));
    eventQueue.push(std::make_shared<SimulationEndEvent>(40000));
    ctx->resetContext(startTime);
    ctx->setPersonLocation("Max", "Office");

    // ================================================================
    //  Step 1: SimulationStart
    // ================================================================
    auto e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::SIMULATION_START);
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "IMVS_Dock");
    EXPECT_FALSE(ctx->getRobot()->isDriving());

    // ================================================================
    //  Step 2: StopDrive (initial, at Dock — pushed by SimStart)
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "IMVS_Dock");
    EXPECT_FALSE(ctx->getRobot()->isDriving());
    // BT ticks: Idle, no pending -> Docking -> already at dock -> EnterIdle
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);

    // ================================================================
    //  Step 3: MissionDispatch
    //  BT accepts the mission, pushes MissionStartEvent
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::MISSION_DISPATCH);
    EXPECT_EQ(appt->state, des::MissionState::IN_PROGRESS);

    // ================================================================
    //  Step 4: MissionStart -> Robot enters SearchState
    //  BT: HasNextLocation -> MoveToNextLocation -> pushes StartDrive
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::MISSION_START);
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::SEARCHING);

    // ================================================================
    //  Step 5: StartDrive to Office (person's first location)
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::START_DRIVE);
    EXPECT_TRUE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getRobot()->getTargetLocation(), "Office");

    // ================================================================
    //  Step 6: StopDrive at Office
    //  BT: ScanLocation finds Max -> StartAccompanyConversation
    //  Robot is still SEARCHING — state changes when conversation event runs
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "Office");
    EXPECT_FALSE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::SEARCHING);

    // ================================================================
    //  Step 7: StartFoundPersonConversation
    //  Now robot enters CONVERSATE state, pushes ConvComplete event
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::START_FOUND_PERSON_CONV);
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::CONVERSATE);

    // ================================================================
    //  Step 8: FoundPersonConversationComplete (probability=1.0 -> Success)
    //  BT: WasConversationSuccessful -> StartAccompanyAction
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::FOUND_PERSON_CONV_COMPLETE);
    EXPECT_EQ(ctx->getRobot()->getState()->getResult(), des::Result::SUCCESS);

    // ================================================================
    //  Step 9: StartAccompany -> AccompanyState, pushes StartDrive
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::START_ACCOMPANY);
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::ACCOMPANY);

    // ================================================================
    //  Step 10: StartDrive to MeetingRoom (with person)
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::START_DRIVE);
    EXPECT_TRUE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getRobot()->getTargetLocation(), "MeetingRoom");

    // ================================================================
    //  Step 11: StopDrive at MeetingRoom
    //  ACCOMPANY mode -> person moved to MeetingRoom (PersonTransition pushed)
    //  BT: ArrivedWithPerson -> StartDropOffConversation -> CONVERSATE
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "MeetingRoom");
    EXPECT_FALSE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getPersonLocation("Max"), "MeetingRoom");
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::CONVERSATE);

    // ================================================================
    //  Steps 12-13: PersonTransition (accompany arrival) + StartDropOffConversation
    //  Same timestamp, order may vary.
    // ================================================================
    {
        bool seenPersonTransition = false;
        bool seenStartDropOff = false;
        for (int i = 0; i < 2; ++i) {
            e = step(*ctx);
            ASSERT_NE(e, nullptr);
            if (e->getType() == des::EventType::PERSON_TRANSITION) seenPersonTransition = true;
            if (e->getType() == des::EventType::START_DROP_OFF_CONV) seenStartDropOff = true;
        }
        EXPECT_TRUE(seenPersonTransition) << "Expected PersonTransition (accompany arrival)";
        EXPECT_TRUE(seenStartDropOff) << "Expected StartDropOffConversation";
    }

    // ================================================================
    //  Step 14: DropOffConversationComplete (Success)
    //  BT: CompleteMission -> COMPLETED, state -> IDLE
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::DROP_OFF_CONV_COMPLETE);
    EXPECT_EQ(appt->state, des::MissionState::COMPLETED);
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);

    // ================================================================
    //  Step 15: MissionComplete
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::MISSION_COMPLETE);
    EXPECT_FALSE(observer->missionCompletions.empty());
    EXPECT_EQ(observer->missionCompletions.back().second, des::MissionState::COMPLETED);

    // ================================================================
    //  Step 16: StartDrive back to Dock
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::START_DRIVE);
    EXPECT_TRUE(ctx->getRobot()->isDriving());

    // ================================================================
    //  Step 17: StopDrive at Dock -> robot is home
    // ================================================================
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "IMVS_Dock");
    EXPECT_FALSE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);

    // ================================================================
    //  Remaining events: PersonTransition follow-ups + SimulationEnd
    //  Drain until SimulationEnd, then verify final state.
    // ================================================================
    bool seenSimEnd = false;
    while (!eventQueue.empty()) {
        e = step(*ctx);
        if (e->getType() == des::EventType::SIMULATION_END) {
            seenSimEnd = true;
            EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);
        }
    }
    EXPECT_TRUE(seenSimEnd) << "SimulationEnd event must be processed";

    // ================================================================
    //  Final assertions
    // ================================================================
    EXPECT_TRUE(eventQueue.empty());
    EXPECT_EQ(appt->state, des::MissionState::COMPLETED);

    // Verify key state transitions happened in correct order
    bool foundSearch = false, foundAccompany = false, foundComplete = false;
    for (const auto& [time, state] : observer->stateChanges) {
        if (!foundSearch && state == des::RobotStateType::SEARCHING) foundSearch = true;
        if (foundSearch && !foundAccompany && state == des::RobotStateType::ACCOMPANY) foundAccompany = true;
        if (foundAccompany && !foundComplete && state == des::RobotStateType::IDLE) foundComplete = true;
    }
    EXPECT_TRUE(foundSearch) << "Missing SEARCHING state";
    EXPECT_TRUE(foundAccompany) << "Missing ACCOMPANY state after SEARCHING";
    EXPECT_TRUE(foundComplete) << "Missing final IDLE state after ACCOMPANY";
}
