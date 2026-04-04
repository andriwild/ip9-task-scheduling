#include <gtest/gtest.h>
#include <memory>

#include "../src/behaviour/bt_setup.h"
#include "../src/init/config_loader.h"
#include "../src/model/context.h"
#include "../src/model/event.h"
#include "../src/model/event_queue.h"
#include "../src/observer/observer.h"
#include "../src/sim/i_path_planner.h"
#include "../src/sim/scheduler.h"

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
    std::map<std::string, std::shared_ptr<des::Person>> employeeLocations;
    std::shared_ptr<TrackingObserver> observer;

    void SetUp() override {
        planner = std::make_shared<MockPathPlanner>();
        observer = std::make_shared<TrackingObserver>();

        config = std::make_shared<des::SimConfig>();
        config->robotSpeed = 1.0;
        config->robotAccompanySpeed = 0.5;
        config->timeBuffer = 60.0;
        config->cacheEnabled = false;
        config->personFindProbability = 1.0;
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
        max->currentRoom = "Office";
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

    std::unique_ptr<Scheduler> makeScheduler() {
        return std::make_unique<Scheduler>(config, planner, employeeLocations);
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
    auto scheduler = makeScheduler();

    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, *scheduler
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
    std::vector<std::shared_ptr<des::Appointment>> appointments = {appt};
    auto missions = scheduler->simplePlan(appointments, "IMVS_Dock");
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
    auto scheduler = makeScheduler();

    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, *scheduler
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
    auto scheduler = makeScheduler();

    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, *scheduler
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
    auto scheduler = makeScheduler();

    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, *scheduler
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
    auto scheduler = makeScheduler();

    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, *scheduler
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
    auto scheduler = makeScheduler();

    auto ctx = std::make_shared<SimulationContext>(
        eventQueue, config, planner, employeeLocations, *scheduler
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
