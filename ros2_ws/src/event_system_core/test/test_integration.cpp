#include <algorithm>

#include <gtest/gtest.h>
#include <memory>

#include "../src/behaviour/bt_setup.h"
#include "../src/init/config_loader.h"
#include "engine/context.h"
#include "engine/event.h"
#include "engine/event_queue.h"
#include "../src/observer/observer.h"
#include "../src/sim/i_path_planner.h"
#include "../src/sim/scheduler.h"
#include "../src/plugins/accompany/accompany_order.h"
#include "../src/plugins/accompany/accompany_plugin.h"
#include "../src/plugins/accompany/states.h"
#include "../src/plugins/order_registry.h"
#include "util/constants.h"

class MockPathPlanner : public des::IPathPlanner {
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

class TrackingObserver : public des::IObserver {
public:
    std::vector<std::pair<int, des::EventType>> events;
    // Each state-change carries both the structural category and the
    // plugin-supplied name so tests can assert either dimension.
    struct StateChange { int time; des::RobotStateType type; std::string name; };
    std::vector<StateChange> stateChanges;

    bool sawEvent(const des::EventType type) const {
        return std::any_of(events.begin(), events.end(), [type](const auto& e) {
            return e.second == type;
        });
    }

    std::string getName() override { return "TrackingObserver"; }

    void onEvent(int time, des::EventType type, const std::string&, bool, bool, const std::string&, int) override {
        events.emplace_back(time, type);
    }

    void onStateChanged(int time, const des::RobotStateType& type, const std::string& name, des::BatteryProps) override {
        stateChanges.push_back({time, type, name});
    }


};

static std::shared_ptr<des::AccompanyOrder> makeAccompanyOrder(
        int id,
        const std::string& person,
        const std::string& room,
        int appointmentTime,
        const std::string& description = "Test") {
    auto o = std::make_shared<des::AccompanyOrder>();
    o->id = id;
    o->type = "accompany";
    o->personName = person;
    o->roomName = room;
    o->deadline = appointmentTime;
    o->description = description;
    return o;
}

class IntegrationTest : public ::testing::Test {
protected:
    des::EventQueue eventQueue;
    std::shared_ptr<MockPathPlanner> planner;
    std::shared_ptr<des::SimConfig> config;
    des::PersonMap employees;
    des::RoomMap roomMap;
    std::shared_ptr<TrackingObserver> observer;

    static void SetUpTestSuite() {
        static bool registered = false;
        if (!registered) {
            des::OrderRegistry::instance().registerPlugin(std::make_unique<des::AccompanyOrderPlugin>());
            registered = true;
        }
    }

    void SetUp() override {
        planner = std::make_shared<MockPathPlanner>();
        observer = std::make_shared<TrackingObserver>();

        config = std::make_shared<des::SimConfig>();
        config->robotSpeed = 1.0;
        config->timeBuffer = 60.0;
        config->cacheEnabled = false;
        config->driveDelayMedian = 0.0;

        // Accompany-specific params live on the plugin now. Deterministic
        // (std=0, probability=1.0) so the integration scenario is reproducible.
        des::OrderRegistry::instance().get(des::AccompanyOrderPlugin::kTypeName).loadConfig(nlohmann::json{
            {"accompany_speed",            0.5},
            {"conversation_probability",   1.0},
            {"conversation_duration_mean", 30.0},
            {"conversation_duration_std",  0.0},
            {"appointment_duration",       1800.0},
        });
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

        // Location areas so ScanAera produces a non-zero scanTime
        roomMap.emplace("IMVS_Dock", des::Room("IMVS_Dock", {}, 50.0));
        roomMap.emplace("Office", des::Room("Office", {}, 50.0));
        roomMap.emplace("MeetingRoom", des::Room("MeetingRoom", {}, 50.0));

    }

    // Employee
    des::PersonList makePeople() {
        auto max = std::make_unique<des::Person>();
        max->firstName = "Max";
        max->lastName = "Mustermann";
        max->workplace = "Office";
        max->roomLabels = {"Office"};
        max->transitionMatrix = {{1.0}};
        max->arrivalTime = 28800;
        max->departureTime = 61200;
        employees["Max"] = max.get();

        des::PersonList people;
        people.push_back(std::move(max));
        return people;
    }

    // Run the event loop similar to main.cpp
    void runEventLoop(des::SimulationContext& ctx, int maxEvents = 200) {
        int processed = 0;
        while (!eventQueue.empty() && processed < maxEvents) {
            auto e = eventQueue.top();
            eventQueue.pop();
            ctx.advanceTime(e->time);
            ctx.executeEvent(e);
            processed++;
        }
    }

    // Execute the next event from the queue and return it
    std::shared_ptr<des::IEvent> step(des::SimulationContext& ctx) {
        if (eventQueue.empty()) return nullptr;
        auto e = eventQueue.top();
        eventQueue.pop();
        ctx.advanceTime(e->time);
        ctx.executeEvent(e);
        return e;
    }

};

// --- Config Roundtrip (kept here for completeness with integration) ---

TEST(ConfigRoundtrip, SaveAndReloadPreservesAllFields) {
    auto original = des::ConfigLoader::loadSimConfig(TEST_FIXTURES_DIR + std::string("/test_sim_config.json"));
    ASSERT_TRUE(original.has_value());

    std::string tmpFile = "/tmp/test_roundtrip_integration.json";
    auto ptr = std::make_shared<des::SimConfig>(*original);
    ASSERT_TRUE(des::ConfigLoader::saveSimConfig(tmpFile, ptr));

    auto reloaded = des::ConfigLoader::loadSimConfig(tmpFile);
    ASSERT_TRUE(reloaded.has_value());

    // Verify all distribution-related fields survive the roundtrip
    EXPECT_EQ(des::distributionTypeToString(original->arrivalDistribution),
              des::distributionTypeToString(reloaded->arrivalDistribution));
    EXPECT_EQ(des::distributionTypeToString(original->departureDistribution),
              des::distributionTypeToString(reloaded->departureDistribution));

    std::filesystem::remove(tmpFile);
}

TEST_F(IntegrationTest, ContextIsDestroyedAfterDroppingLastReference) {
    std::weak_ptr<des::SimulationContext> observer;
    {
        auto ctx = std::make_shared<des::SimulationContext>(
            eventQueue, config, planner, makePeople(), roomMap
        );
        ctx->setBehaviorTree(des::setupBehaviorTree(ctx.get()));
        observer = ctx;
        EXPECT_FALSE(observer.expired());
    }
    EXPECT_TRUE(observer.expired());
}

// --- Event Loop: full scenario ---

TEST_F(IntegrationTest, SingleMissionCompletesSuccessfully) {
    auto ctx = std::make_shared<des::SimulationContext>(
        eventQueue, config, planner, makePeople(), roomMap
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(des::setupBehaviorTree(ctx.get()));

    // Create a simple scenario: one appointment
    auto order = makeAccompanyOrder(0, "Max", "MeetingRoom", 36000, "Test Meeting");

    // Schedule mission (same as des::IAppRunner::createMissionQueue)
    des::OrderList orders = {order};
    auto missions = ctx->getScheduler().createMissionDispatchEvents(orders, "IMVS_Dock");
    for (auto& m : missions) {
        m->time = m->time - config->timeBuffer;
        eventQueue.push(m);
    }

    // Add simulation start/end
    int startTime = eventQueue.getFirstEventTime() - ONE_HOUR;
    int endTime = 40000;
    eventQueue.push(std::make_shared<des::SimulationStartEvent>(startTime));
    eventQueue.push(std::make_shared<des::SimulationEndEvent>(endTime));

    ctx->resetContext(startTime);
    ctx->setPersonLocation("Max", "Office");
    runEventLoop(*ctx);

    // Verify: mission was completed
    EXPECT_TRUE(observer->sawEvent(des::EventType::MISSION_COMPLETE));
    // Verify: robot went through state changes
    EXPECT_FALSE(observer->stateChanges.empty());
    // Verify: robot moved at least once
    EXPECT_TRUE(observer->sawEvent(des::EventType::STOP_DRIVE));
    // Verify: multiple events were processed
    EXPECT_GT(observer->events.size(), 5u);

    // Verify: state changes include search, accompany and conversate phases
    bool hasSearch = false, hasAccompany = false, hasConversate = false;
    for (const auto& sc : observer->stateChanges) {
        if (sc.name == "search")     hasSearch = true;
        if (sc.name == "accompany")  hasAccompany = true;
        if (sc.name == "conversate") hasConversate = true;
    }
    EXPECT_TRUE(hasSearch);
    EXPECT_TRUE(hasAccompany);
    EXPECT_TRUE(hasConversate);
}

TEST_F(IntegrationTest, PersonOutsideTheVisibilityPolygonIsNotSeen) {
    config->personIdentificationRange = 100.0;
    roomMap.at("Office").m_footprint = {
        des::Point{10.0, 10.0, 0.0},
        des::Point{11.0, 10.0, 0.0},
        des::Point{11.0, 11.0, 0.0},
        des::Point{10.0, 11.0, 0.0}
    };

    auto ctx = std::make_shared<des::SimulationContext>(
        eventQueue, config, planner, makePeople(), roomMap
    );
    ctx->setPersonLocation("Max", "Office");
    ctx->getRobot()->setLocation("Office");
    ctx->robotMovedTo(des::Point{0.0, 0.0, 0.0}, 0.0);

    EXPECT_TRUE(ctx->robotSeesPerson("Max"));

    ctx->getRobot()->setVisibility({
        des::Point{0.0, 0.0, 0.0},
        des::Point{5.0, 0.0, 0.0},
        des::Point{5.0, 5.0, 0.0},
        des::Point{0.0, 5.0, 0.0}
    });
    EXPECT_FALSE(ctx->robotSeesPerson("Max"));

    ctx->getRobot()->setVisibility({
        des::Point{0.0, 0.0, 0.0},
        des::Point{20.0, 0.0, 0.0},
        des::Point{20.0, 20.0, 0.0},
        des::Point{0.0, 20.0, 0.0}
    });
    EXPECT_TRUE(ctx->robotSeesPerson("Max"));
}

TEST_F(IntegrationTest, EventLoopDrainsQueue) {
    auto ctx = std::make_shared<des::SimulationContext>(
        eventQueue, config, planner, makePeople(), roomMap
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(des::setupBehaviorTree(ctx.get()));

    // Minimal scenario: just start and end
    eventQueue.push(std::make_shared<des::SimulationStartEvent>(1000));
    eventQueue.push(std::make_shared<des::SimulationEndEvent>(2000));
    ctx->resetContext(1000);

    runEventLoop(*ctx);

    // Queue should be empty after processing
    EXPECT_TRUE(eventQueue.empty());
}

TEST_F(IntegrationTest, MissionDispatchWithoutPriorStartIsPending) {
    auto ctx = std::make_shared<des::SimulationContext>(
        eventQueue, config, planner, makePeople(), roomMap
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(des::setupBehaviorTree(ctx.get()));

    auto order = makeAccompanyOrder(0, "Max", "MeetingRoom", 36000);

    // Start simulation, then dispatch mission
    eventQueue.push(std::make_shared<des::SimulationStartEvent>(30000));
    eventQueue.push(std::make_shared<des::MissionDispatchEvent>(34000, order));
    eventQueue.push(std::make_shared<des::SimulationEndEvent>(40000));
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
    auto ctx = std::make_shared<des::SimulationContext>(
        eventQueue, config, planner, makePeople(), roomMap
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(des::setupBehaviorTree(ctx.get()));

    // Run some events to change state
    eventQueue.push(std::make_shared<des::SimulationStartEvent>(1000));
    auto order = makeAccompanyOrder(0, "Max", "MeetingRoom", 5000);
    eventQueue.push(std::make_shared<des::MissionDispatchEvent>(2000, order));
    ctx->resetContext(1000);

    // Process start event
    auto e = eventQueue.top();
    eventQueue.pop();
    ctx->advanceTime(e->time);
    ctx->executeEvent(e);

    EXPECT_EQ(ctx->getTime(), 1000);

    // Reset
    ctx->resetContext(5000);

    EXPECT_EQ(ctx->getTime(), 5000);
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), config->dockLocation);
    EXPECT_FALSE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getOrderPtr(), nullptr);
    EXPECT_FALSE(ctx->hasScheduledOrder());
}

TEST_F(IntegrationTest, ResetContextAllowsRerun) {
    auto ctx = std::make_shared<des::SimulationContext>(
        eventQueue, config, planner, makePeople(), roomMap
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(des::setupBehaviorTree(ctx.get()));

    // First run
    eventQueue.push(std::make_shared<des::SimulationStartEvent>(1000));
    eventQueue.push(std::make_shared<des::SimulationEndEvent>(2000));
    ctx->resetContext(1000);
    runEventLoop(*ctx);

    size_t firstRunEvents = observer->events.size();
    EXPECT_GT(firstRunEvents, 0u);

    // Reset and run again
    ctx->resetContext(3000);
    eventQueue.push(std::make_shared<des::SimulationStartEvent>(3000));
    eventQueue.push(std::make_shared<des::SimulationEndEvent>(4000));
    runEventLoop(*ctx);

    // Should have processed more events
    EXPECT_GT(observer->events.size(), firstRunEvents);
}

// --- Observer integration ---

TEST_F(IntegrationTest, ObserverReceivesEventsInOrder) {
    auto ctx = std::make_shared<des::SimulationContext>(
        eventQueue, config, planner, makePeople(), roomMap
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(des::setupBehaviorTree(ctx.get()));

    eventQueue.push(std::make_shared<des::SimulationStartEvent>(1000));
    eventQueue.push(std::make_shared<des::SimulationEndEvent>(5000));
    ctx->resetContext(1000);

    runEventLoop(*ctx);

    // Events should be in chronological order
    for (size_t i = 1; i < observer->events.size(); ++i) {
        EXPECT_GE(observer->events[i].first, observer->events[i - 1].first)
            << "Event at index " << i << " is out of order";
    }
}

// --- Step-by-step event execution ---

TEST_F(IntegrationTest, StepByStepSingleMission) {
    auto ctx = std::make_shared<des::SimulationContext>(
        eventQueue, config, planner, makePeople(), roomMap
    );
    ctx->addObserver(observer);
    ctx->setBehaviorTree(des::setupBehaviorTree(ctx.get()));

    auto order = makeAccompanyOrder(0, "Max", "MeetingRoom", 36000, "Dokument abholen");

    des::OrderList orders = {order};
    auto missions = ctx->getScheduler().createMissionDispatchEvents(orders, "IMVS_Dock");
    for (auto& m : missions) {
        m->time = m->time - config->timeBuffer;
        eventQueue.push(m);
    }

    int startTime = eventQueue.getFirstEventTime() - ONE_HOUR;
    eventQueue.push(std::make_shared<des::SimulationStartEvent>(startTime));
    eventQueue.push(std::make_shared<des::SimulationEndEvent>(40000));
    ctx->resetContext(startTime);
    ctx->setPersonLocation("Max", "Office");

    // Step 1: SimulationStart
    auto e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::SIMULATION_START);
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "IMVS_Dock");
    EXPECT_FALSE(ctx->getRobot()->isDriving());

    // Step 2: StopDrive (initial, at Dock — pushed by SimStart)
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "IMVS_Dock");
    EXPECT_FALSE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);

    // Step 3: MissionDispatch — order moves into the pending queue but stays
    // PENDING; the IN_PROGRESS transition happens in the plugin's StartXxxEvent
    // (accompany has no such event so it goes straight from PENDING to COMPLETED).
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::MISSION_DISPATCH);
    EXPECT_EQ(order->state, des::OrderState::PENDING);

    // Step 4: MissionStart -> des::Robot enters des::SearchState
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::MISSION_START);
    EXPECT_EQ(ctx->getRobot()->getState()->getName(), "search");

    // Step 5: StartDrive to Office — the dock is excluded from the search, so it is not scanned
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::START_DRIVE);
    EXPECT_TRUE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getRobot()->getTargetLocation(), "Office");

    // Step 6: StopDrive at Office
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "Office");
    EXPECT_FALSE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getRobot()->getState()->getName(), "search");

    // Step 7: Scan at Office — person found
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::SCAN);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "Office");
    EXPECT_TRUE(ctx->getRobot()->isPersonVisible());

    // Step 8: StartFoundPersonConversation
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::CONVERSATION_START);
    EXPECT_EQ(ctx->getRobot()->getState()->getName(), "conversate");

    // Step 9: FoundPersonConversationComplete
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::CONVERSATION_END);
    EXPECT_EQ(ctx->getRobot()->getState()->getResult(), des::Result::SUCCESS);

    // Step 10: StartAccompany
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::START_ACCOMPANY);
    EXPECT_EQ(ctx->getRobot()->getState()->getName(), "accompany");

    // Steps 11-12: PersonAccompanyDeparture + StartDrive (same time, order not guaranteed)
    {
        bool seenDeparture = false;
        bool seenStartDrive = false;
        for (int i = 0; i < 2; ++i) {
            e = step(*ctx);
            ASSERT_NE(e, nullptr);
            if (e->getType() == des::EventType::PERSON_ACCOMPANY_DEPARTURE) seenDeparture = true;
            if (e->getType() == des::EventType::START_DRIVE) seenStartDrive = true;
        }
        EXPECT_TRUE(seenDeparture) << "Expected PersonAccompanyDeparture at accompany start";
        EXPECT_TRUE(seenStartDrive) << "Expected StartDrive to the room waypoint";
    }
    EXPECT_TRUE(ctx->getRobot()->isDriving());

    // Steps 13-14: StopDrive at the waypoint of Office, then StartDrive to MeetingRoom
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "Office");

    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::START_DRIVE);
    EXPECT_TRUE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getRobot()->getTargetLocation(), "MeetingRoom");

    // Step 15: StopDrive at MeetingRoom
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "MeetingRoom");
    EXPECT_FALSE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getPersonLocation("Max"), "MeetingRoom");
    EXPECT_EQ(ctx->getRobot()->getState()->getName(), "accompany");

    // Steps 16-17: PersonAccompanyArrived + des::StartDropOffConversation
    {
        bool seenArrived = false;
        bool seenStartDropOff = false;
        for (int i = 0; i < 2; ++i) {
            e = step(*ctx);
            ASSERT_NE(e, nullptr);
            if (e->getType() == des::EventType::PERSON_ACCOMPANY_ARRIVED) seenArrived = true;
            if (e->getType() == des::EventType::CONVERSATION_START) seenStartDropOff = true;
        }
        EXPECT_TRUE(seenArrived) << "Expected PersonAccompanyArrived (accompany arrival)";
        EXPECT_TRUE(seenStartDropOff) << "Expected StartDropOffConversation";
    }
    EXPECT_EQ(ctx->getRobot()->getState()->getName(), "conversate");

    // Step 18: DropOffConversationComplete (Success)
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::CONVERSATION_END);
    EXPECT_EQ(order->state, des::OrderState::COMPLETED);
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);

    // Step 19: MissionComplete
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::MISSION_COMPLETE);
    EXPECT_TRUE(observer->sawEvent(des::EventType::MISSION_COMPLETE));
    EXPECT_EQ(order->state, des::OrderState::COMPLETED);

    // Step 20: StartDrive back to Dock
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::START_DRIVE);
    EXPECT_TRUE(ctx->getRobot()->isDriving());

    // Step 21: StopDrive at Dock
    e = step(*ctx);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx->getRobot()->getLocation(), "IMVS_Dock");
    EXPECT_FALSE(ctx->getRobot()->isDriving());
    EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);

    // Drain remaining events
    bool seenSimEnd = false;
    while (!eventQueue.empty()) {
        e = step(*ctx);
        if (e->getType() == des::EventType::SIMULATION_END) {
            seenSimEnd = true;
            EXPECT_EQ(ctx->getRobot()->getStateType(), des::RobotStateType::IDLE);
        }
    }
    EXPECT_TRUE(seenSimEnd) << "SimulationEnd event must be processed";

    EXPECT_TRUE(eventQueue.empty());
    EXPECT_EQ(order->state, des::OrderState::COMPLETED);

    // Verify key state transitions happened in correct order
    bool foundSearch = false, foundAccompany = false, foundComplete = false;
    for (const auto& sc : observer->stateChanges) {
        if (!foundSearch && sc.name == "search") foundSearch = true;
        if (foundSearch && !foundAccompany && sc.name == "accompany") foundAccompany = true;
        if (foundAccompany && !foundComplete && sc.type == des::RobotStateType::IDLE) foundComplete = true;
    }
    EXPECT_TRUE(foundSearch) << "Missing search state";
    EXPECT_TRUE(foundAccompany) << "Missing accompany state after search";
    EXPECT_TRUE(foundComplete) << "Missing final IDLE state after accompany";
}
