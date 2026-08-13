#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
#include <random>
#include <stdexcept>

#include "engine/context.h"
#include "engine/event.h"
#include "engine/contracts/i_sim_context.h"
#include "../src/model/robot.h"
#include "../src/model/robot_state.h"
#include "../src/plugins/accompany/accompany_order.h"
#include "../src/plugins/accompany/accompany_plugin.h"
#include "../src/plugins/accompany/states.h"
#include "../src/plugins/order_registry.h"
#include "../src/plugins/accompany/events/abort_search_event.h"
#include "../src/plugins/accompany/events/start_accompany_event.h"
#include "../src/engine/event/conversation_event.h"
#include "../src/plugins/accompany/events/appointment_end_event.h"
#include "../src/plugins/accompany/events/scan_point_event.h"
#include "util/constants.h"

class MockSimContext : public des::ISimContext {
public:
    // Tracking
    des::EventList pushedEvents;
    std::vector<std::string> notifiedEvents;
    int tickCount = 0;
    bool completeOrderCalled = false;
    std::map<std::string, std::string> blackboard;

    // Configurable state
    std::shared_ptr<des::Robot> robot;
    des::OrderPtr currentOrder;
    std::shared_ptr<des::SimConfig> simConfig;
    des::PersonMap employees;
    std::map<std::string, std::string> personLocations;
    des::OrderList pendingMissions;
    des::OrderList m_backgroundMissions;
    std::optional<int> nextScheduledDispatchTime;
    des::OrderPtr m_nextScheduledOrder;
    std::optional<int> m_simulationEndTime;
    int currentTime = 0;
    mutable std::mt19937 m_rng{42};
    mutable std::mt19937 m_robotRng{43};

    MockSimContext() {
        simConfig = std::make_shared<des::SimConfig>();
        simConfig->robotSpeed = 1.0;
        simConfig->batteryCapacity = 100.0;
        simConfig->initialBatteryCapacity = 80.0;
        simConfig->lowBatteryThreshold = 20.0;
        simConfig->fullBatteryThreshold = 95.0;
        simConfig->driveTimeStd = 0.0;
        simConfig->timeBuffer = 60.0;
        simConfig->energyConsumptionDrive = 0.1;
        simConfig->energyConsumptionBase = 0.01;
        simConfig->chargingRate = 0.5;
        simConfig->dockLocation = "Dock";
        simConfig->cacheEnabled = false;

        robot = std::make_shared<des::Robot>(simConfig);
    }

    int getTime() const override { return currentTime; }

    void pushEvent(const std::shared_ptr<des::IEvent>& event) override {
        pushedEvents.push_back(event);
    }

    void startActivity(const std::shared_ptr<des::IEvent>& event) override {
        // Mock treats startActivity as a regular push — in-flight tracking
        // isn't exercised by unit-level event tests.
        pushedEvents.push_back(event);
    }

    void tickBT() override { tickCount++; }

    void setBTBlackboard(const std::string& key, const std::string& value) override {
        blackboard[key] = value;
    }

    des::Robot* getRobot() const override { return robot.get(); }

    void changeRobotState(std::unique_ptr<des::RobotState> newState) const override {
        robot->changeState(std::move(newState), currentTime);
    }

    void robotMoved(const std::string& location, double /*distance*/) const override {
        robot->setLocation(location);
    }
    void robotMovedTo(const des::Point& position, double /*distance*/ = 0.0) const override {
        robot->setPosition(position);
    }

    des::Journey scheduleArrival(const std::string& target) const override {
        if (robot->getLocation() == target) {
            return {0.0, 0.0};
        }
        return {10.0, 5.0};
    }

    double mockDistance = 5.0;
    std::optional<double> getDistance(const std::string& /*from*/, const std::string& /*to*/) const override {
        return mockDistance;
    }

    const des::Scheduler& getScheduler() const override {
        throw std::runtime_error("MockSimContext::getScheduler not implemented");
    }

    void notifyEvent(const des::IEvent& event) const override {
        const_cast<MockSimContext*>(this)->notifiedEvents.push_back(event.getName());
    }

    void notifyBatteryChanged() const override {}
    void notifyChargeStarted() const override {}

    bool pushInterrupt(const des::OrderPtr& /*order*/) override { return true; }
    void popInterrupt(const des::OrderPtr& /*completedOrder*/) override {}
    bool hasActiveInterrupt() const override { return false; }

    void setOrderPtr(const des::OrderPtr& order) override {
        currentOrder = order;
    }

    des::OrderPtr getOrderPtr() const override {
        return currentOrder;
    }

    void updateOrderState(const des::OrderState& newState) override {
        if (currentOrder) currentOrder->state = newState;
    }

    void addScheduledOrder(const des::OrderPtr order) override {
        pendingMissions.push_back(order);
    }

    bool hasScheduledOrder() const override { return !pendingMissions.empty(); }

    des::OrderPtr nextScheduledOrder() override {
        return pendingMissions.empty() ? nullptr : pendingMissions.front();
    }

    des::OrderPtr popScheduledOrder() override {
        if (pendingMissions.empty()) return nullptr;
        auto front = pendingMissions.front();
        pendingMissions.erase(pendingMissions.begin());
        return front;
    }

    void addBackgroundOrder(const des::OrderPtr order) override {
        m_backgroundMissions.push_back(order);
    }
    bool hasBackgroundOrder() const override { return !m_backgroundMissions.empty(); }
    des::OrderPtr acceptFeasibleBackgroundOrder() override {
        if (m_backgroundMissions.empty()) return nullptr;
        auto order = m_backgroundMissions.front();
        m_backgroundMissions.erase(m_backgroundMissions.begin());
        currentOrder = order;
        return order;
    }
    std::optional<int> getNextScheduledDispatchTime() const override {
        return nextScheduledDispatchTime;
    }
    des::OrderPtr peekNextScheduledOrder() const override { return m_nextScheduledOrder; }
    std::vector<des::OrderPtr> peekScheduledOrdersUntil(int /*untilTime*/) const override {
        if (!m_nextScheduledOrder) {
            return {};
        }
        return { m_nextScheduledOrder };
    }
    std::optional<int> getSimulationEndTime() const override { return m_simulationEndTime; }

    void completeOrder(const des::OrderPtr& /*order*/) override {
        completeOrderCalled = true;
    }

    void publishMission(const des::OrderPtr& /*order*/, int /*time*/) override {}

    bool hasEmployee(const std::string& person) const override {
        return employees.contains(person);
    }

    des::Person* getPersonByName(const std::string& person) const override {
        return employees.at(person);
    }

    des::PersonList people;
    const des::PersonList& getAllPersons() const override {
        return people;
    }

    std::shared_ptr<des::SimConfig> getConfig() const override { return simConfig; }
    std::mt19937& worldRng() const override { return m_rng; }
    std::mt19937& robotRng() const override { return m_robotRng; }

    std::string getPersonLocation(const std::string& name) const override {
        return personLocations.at(name);
    }
    const std::map<std::string, std::string>& getAllPersonLocations() const override {
        return personLocations;
    }
    void setPersonLocation(const std::string& name, const std::string& room) override {
        personLocations[name] = room;
    }
    std::optional<des::Point> getPersonPosition(const std::string& /*name*/) const override {
        return std::nullopt;
    }
    des::RoomMap rooms;
    const des::Room& room(const std::string& name) const override {
        static const des::Room none{"", des::Point{}, 0.0};
        const auto it = rooms.find(name);
        if (it == rooms.end()) {
            return none;
        }
        return it->second;
    }

    bool robotSeesPerson(const std::string& name) const override {
        auto it = personLocations.find(name);
        if (it == personLocations.end()) {
            return false;
        }
        return it->second == robot->getLocation();
    }

    bool robotRecognizesPerson(const std::string& name) const override {
        return robotSeesPerson(name);
    }

    std::map<std::pair<std::string, std::string>, int> lastServicedMap;
    std::optional<int> lastServiced(const std::string& room, const std::string& type) const override {
        auto it = lastServicedMap.find({room, type});
        return it == lastServicedMap.end() ? std::nullopt : std::optional<int>(it->second);
    }
    void recordServiced(const std::string& room, const std::string& type, int time) override {
        lastServicedMap[{room, type}] = time;
    }

    std::vector<std::string> roomNamesList;
    std::vector<std::string> roomNames() const override {
        return roomNamesList;
    }

};

static bool pluginsRegistered = [] {
    des::OrderRegistry::instance().registerPlugin(std::make_unique<des::AccompanyOrderPlugin>());
    return true;
}();

static std::shared_ptr<des::AccompanyOrder> makeAccompanyOrder(
        int id,
        const std::string& person,
        const std::string& room = "Room1",
        int appointmentTime = 36000,
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

static des::Polygon makeSquare(const double x0, const double y0, const double x1, const double y1) {
    return {
        des::Point{x0, y0, 0.0},
        des::Point{x1, y0, 0.0},
        des::Point{x1, y1, 0.0},
        des::Point{x0, y1, 0.0}
    };
}

// --- Visibility wiring ---

TEST(EventExecute, SearchDriveCarriesTheVisibilityOfItsTourPoint) {
    MockSimContext ctx;

    des::Room room{ "Office", des::Point{}, 0.0 };
    room.m_tour.m_path = { des::Point{1.0, 1.0, 0.0}, des::Point{2.0, 2.0, 0.0} };
    room.m_tour.m_visPolys = { makeSquare(0.0, 0.0, 3.0, 3.0), makeSquare(2.0, 2.0, 5.0, 5.0) };
    ctx.rooms.emplace("Office", std::move(room));
    ctx.robot->setLocation("Office");
    ctx.robot->setSpeed(1.0);

    const auto order = makeAccompanyOrder(1, "Max", "Office");
    const des::RoomTour& tour = ctx.room("Office").m_tour;
    des::requestDrive(ctx, tour.m_path[0], tour.visibilityAt(0), std::make_shared<des::ScanPointEvent>(1000, order));

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    const auto startDrive = ctx.pushedEvents.back();
    ctx.pushedEvents.clear();
    startDrive->execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    const auto stopDrive = ctx.pushedEvents.back();
    ctx.pushedEvents.clear();
    stopDrive->execute(ctx);

    EXPECT_DOUBLE_EQ(ctx.robot->getPosition().m_x, 1.0);
    ASSERT_EQ(ctx.robot->getVisibility().size(), 4u);
    EXPECT_DOUBLE_EQ(ctx.robot->getVisibility()[2].m_x, 3.0);
    EXPECT_DOUBLE_EQ(ctx.robot->getVisibility()[2].m_y, 3.0);
}

TEST(EventExecute, ScanPointLatchesTheTargetAndMarksThePersonBusy) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    ctx.employees["Max"] = person.get();
    ctx.robot->setLocation("Office");
    ctx.personLocations["Max"] = "Office";

    des::ScanPointEvent event(1000, makeAccompanyOrder(1, "Max", "Office"));
    event.execute(ctx);

    EXPECT_TRUE(ctx.robot->isPersonVisible());
    EXPECT_TRUE(person->busy);
    EXPECT_EQ(ctx.tickCount, 1);
}

TEST(EventExecute, ScanPointWithoutSightingLeavesTheRobotSearching) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    ctx.employees["Max"] = person.get();
    ctx.robot->setLocation("Office");
    ctx.personLocations["Max"] = "Kitchen";

    des::ScanPointEvent event(1000, makeAccompanyOrder(1, "Max", "Office"));
    event.execute(ctx);

    EXPECT_FALSE(ctx.robot->isPersonVisible());
    EXPECT_FALSE(person->busy);
    EXPECT_EQ(ctx.tickCount, 1);
}

TEST(EventExecute, ScanPointQueuesBystandersForQuestioning) {
    MockSimContext ctx;
    ctx.simConfig->personDirectionsProbability = 0.5;

    auto target = std::make_shared<des::Person>();
    target->firstName = "Max";
    ctx.employees["Max"] = target.get();
    ctx.robot->setLocation("Office");
    ctx.personLocations["Max"] = "Kitchen";
    ctx.personLocations["Nina"] = "Office";
    ctx.personLocations["Tom"] = "Office";

    auto order = makeAccompanyOrder(1, "Max", "Office");
    des::ScanPointEvent event(1000, order);
    event.execute(ctx);

    ASSERT_EQ(order->pendingAsk.size(), 2u);
    EXPECT_EQ(order->identified.size(), 2u);
}

TEST(EventExecute, ScanPointQueuesEachBystanderOnlyOnce) {
    MockSimContext ctx;
    ctx.simConfig->personDirectionsProbability = 0.5;

    auto target = std::make_shared<des::Person>();
    target->firstName = "Max";
    ctx.employees["Max"] = target.get();
    ctx.robot->setLocation("Office");
    ctx.personLocations["Max"] = "Kitchen";
    ctx.personLocations["Nina"] = "Office";

    auto order = makeAccompanyOrder(1, "Max", "Office");
    des::ScanPointEvent(1000, order).execute(ctx);
    des::ScanPointEvent(1100, order).execute(ctx);

    EXPECT_EQ(order->pendingAsk.size(), 1u);
}

TEST(EventExecute, ScanPointAsksNobodyWhenTheTargetIsInSight) {
    MockSimContext ctx;
    ctx.simConfig->personDirectionsProbability = 0.5;

    auto target = std::make_shared<des::Person>();
    target->firstName = "Max";
    ctx.employees["Max"] = target.get();
    ctx.robot->setLocation("Office");
    ctx.personLocations["Max"] = "Office";
    ctx.personLocations["Nina"] = "Office";

    auto order = makeAccompanyOrder(1, "Max", "Office");
    des::ScanPointEvent event(1000, order);
    event.execute(ctx);

    EXPECT_TRUE(ctx.robot->isPersonVisible());
    EXPECT_TRUE(order->pendingAsk.empty());
    EXPECT_EQ(order->identified.size(), 1u);
}

TEST(EventExecute, ScanPointAsksNobodyWhenDirectionsAreDisabled) {
    MockSimContext ctx;
    ctx.simConfig->personDirectionsProbability = 0.0;

    auto target = std::make_shared<des::Person>();
    target->firstName = "Max";
    ctx.employees["Max"] = target.get();
    ctx.robot->setLocation("Office");
    ctx.personLocations["Max"] = "Kitchen";
    ctx.personLocations["Nina"] = "Office";

    auto order = makeAccompanyOrder(1, "Max", "Office");
    des::ScanPointEvent event(1000, order);
    event.execute(ctx);

    EXPECT_TRUE(order->pendingAsk.empty());
    EXPECT_EQ(order->identified.size(), 1u);
}

TEST(EventExecute, ArrivingInARoomAdoptsItsFirstTourPointVisibility) {
    MockSimContext ctx;

    des::Room room{ "Office", des::Point{}, 0.0 };
    room.m_tour.m_path = { des::Point{1.0, 1.0, 0.0} };
    room.m_tour.m_visPolys = { makeSquare(0.0, 0.0, 3.0, 3.0) };
    ctx.rooms.emplace("Office", std::move(room));

    des::RoomTarget target("Office");
    target.arrive(ctx, 1.0);

    ASSERT_EQ(ctx.robot->getVisibility().size(), 4u);
    EXPECT_DOUBLE_EQ(ctx.robot->getVisibility()[2].m_x, 3.0);
}

TEST(EventExecute, ArrivingInARoomWithoutTourKeepsFullSight) {
    MockSimContext ctx;
    ctx.rooms.emplace("Dock", des::Room{ "Dock", des::Point{}, 0.0 });
    ctx.robot->setVisibility(makeSquare(0.0, 0.0, 3.0, 3.0));

    des::RoomTarget target("Dock");
    target.arrive(ctx, 1.0);

    EXPECT_TRUE(ctx.robot->getVisibility().empty());
}

// --- des::SimulationStartEvent ---

TEST(EventExecute, SimulationStartSetsIdleState) {
    MockSimContext ctx;
    des::SimulationStartEvent event(0);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::IDLE);
    EXPECT_FALSE(ctx.pushedEvents.empty());
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

TEST(EventExecute, SimulationStartPushesStopDriveEvent) {
    MockSimContext ctx;
    des::SimulationStartEvent event(500);
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(ctx.pushedEvents[0]->time, 500);
}

// --- des::SimulationEndEvent ---

TEST(EventExecute, SimulationEndSetsIdleState) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<des::SearchState>(), ctx.currentTime);

    des::SimulationEndEvent event(1000);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::IDLE);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::STOP_DRIVE);
}

// --- des::MissionDispatchEvent ---

TEST(EventExecute, MissionDispatchAddsPendingAndTicksBT) {
    MockSimContext ctx;

    auto order = makeAccompanyOrder(1, "Max");

    des::MissionDispatchEvent event(35000, order);
    event.execute(ctx);

    ASSERT_EQ(ctx.pendingMissions.size(), 1u);
    auto accompany = std::dynamic_pointer_cast<des::AccompanyOrder>(ctx.pendingMissions[0]);
    ASSERT_NE(accompany, nullptr);
    EXPECT_EQ(accompany->personName, "Max");
    EXPECT_EQ(ctx.tickCount, 1);
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

// --- des::MissionStartEvent ---

TEST(EventExecute, MissionStartSeedsSearchFromRoomUniverse) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->workplace = "Office";
    ctx.employees["Max"] = person.get();
    ctx.roomNamesList = {"Office", "Kitchen", "Lab"};
    for (const auto& name : ctx.roomNamesList) {
        des::Room room{ name, des::Point{}, 0.0 };
        room.m_tour = des::RoomTour{ 10.0, { des::Point{}, des::Point{} }, {} };
        ctx.rooms.emplace(name, std::move(room));
    }

    auto order = makeAccompanyOrder(1, "Max", "Room1", 40000);
    ctx.setOrderPtr(order);

    des::MissionStartEvent event(35000, order);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getState()->getName(), "search");
    EXPECT_EQ(ctx.tickCount, 1);

    ASSERT_NE(dynamic_cast<des::SearchState*>(ctx.robot->getState()), nullptr);

    std::vector<std::string> got = order->remainingSearch;
    std::sort(got.begin(), got.end());
    EXPECT_EQ(got, (std::vector<std::string>{"Kitchen", "Lab", "Office"}));
    EXPECT_TRUE(order->scanQueue.empty());
}

// --- des::AbortSearchEvent ---

TEST(EventExecute, AbortSearchReportsReasonWithoutEndingTheMission) {
    MockSimContext ctx;

    auto order = makeAccompanyOrder(1, "Max");
    order->state = des::OrderState::IN_PROGRESS;
    ctx.currentOrder = order;
    ctx.personLocations["Max"] = "OUTDOOR";

    des::AbortSearchEvent event(36000, order);
    event.execute(ctx);

    EXPECT_EQ(order->abortReason, des::SearchAbortReason::OUTSIDE);
    EXPECT_EQ(ctx.currentOrder->state, des::OrderState::IN_PROGRESS);
    EXPECT_TRUE(ctx.pushedEvents.empty());
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

TEST(EventExecute, AbortSearchInBuildingSetsReason) {
    MockSimContext ctx;

    auto order = makeAccompanyOrder(1, "Max");
    order->state = des::OrderState::IN_PROGRESS;
    order->plannedSearch = {"5.2B03"};
    ctx.currentOrder = order;
    ctx.personLocations["Max"] = "5.2B10";

    des::AbortSearchEvent event(36000, order);
    event.execute(ctx);

    EXPECT_EQ(order->abortReason, des::SearchAbortReason::IN_BUILDING_FINDABLE);
}

// --- des::StartAccompanyEvent ---

TEST(EventExecute, StartAccompanyDrivesToTheRoomViaItsWaypoint) {
    MockSimContext ctx;

    des::Room office{ "Office", des::Point{5.0, 0.0, 0.0}, 0.0 };
    office.m_tour.m_path = { des::Point{5.0, 0.0, 0.0}, des::Point{8.0, 0.0, 0.0} };
    ctx.rooms.emplace("Office", std::move(office));
    ctx.robot->setLocation("Office");
    ctx.robot->setPosition(des::Point{8.0, 0.0, 0.0});
    ctx.robot->setSpeed(1.0);

    auto order = makeAccompanyOrder(1, "Max", "MeetingRoom");
    ctx.currentOrder = order;

    des::StartAccompanyEvent event(35500, order);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getState()->getName(), "accompany");
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::START_DRIVE);
    EXPECT_EQ(ctx.pushedEvents[0]->getName(), "Departing: (5.000000, 0.000000)");

    const auto toWaypoint = ctx.pushedEvents.back();
    ctx.pushedEvents.clear();
    toWaypoint->execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    const auto atWaypoint = ctx.pushedEvents.back();
    ctx.pushedEvents.clear();
    atWaypoint->execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getName(), "Departing: MeetingRoom");
}

// --- des::StartConversationEvent ---

static des::ConversationSpec makeConversationSpec(const des::ConversationKind kind,
                                                  const double durationMean = 30.0,
                                                  const double durationStd = 0.0,
                                                  const double successProbability = 1.0) {
    return des::ConversationSpec{ kind, "Max", durationMean, durationStd, successProbability };
}

TEST(EventExecute, StartConversationPushesSuccessWithHighProbability) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    des::StartConversationEvent event(35000, makeConversationSpec(des::ConversationKind::FOUND_PERSON));
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getState()->getName(), "conversate");
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getName(), "Conversation Successful");
}

TEST(EventExecute, StartConversationPushesFailedWithZeroProbability) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    des::StartConversationEvent event(35000, makeConversationSpec(
        des::ConversationKind::FOUND_PERSON, 30.0, 0.0, /*successProbability=*/0.0));
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getName(), "Conversation Failed ");
}

TEST(EventExecute, StartConversationEventTimeIncludesDuration) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    des::StartConversationEvent event(35000, makeConversationSpec(
        des::ConversationKind::FOUND_PERSON, /*durationMean=*/45.0, /*durationStd=*/0.0));
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->time, 35000 + 45);
}

TEST(EventExecute, StartConversationClampsDurationToOneSecond) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    des::StartConversationEvent event(35000, makeConversationSpec(
        des::ConversationKind::DROP_OFF, /*durationMean=*/-10.0, /*durationStd=*/0.0));
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->time, 35000 + 1);
}

TEST(EventExecute, StartConversationPutsKindOnRobotState) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    des::StartConversationEvent event(35000, makeConversationSpec(des::ConversationKind::DROP_OFF));
    event.execute(ctx);

    const auto state = dynamic_cast<des::ConversationState*>(ctx.robot->getState());
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->kind, des::ConversationKind::DROP_OFF);
}

// --- des::EndConversationEvent ---

TEST(EventExecute, EndConversationSetsSuccessAndTicks) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<des::ConversationState>(des::ConversationKind::FOUND_PERSON), ctx.currentTime);

    des::EndConversationEvent event(35030, makeConversationSpec(des::ConversationKind::FOUND_PERSON), true);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getState()->getResult(), des::Result::SUCCESS);
    EXPECT_EQ(ctx.tickCount, 1);
}

TEST(EventExecute, EndConversationSetsFailureAndTicks) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<des::ConversationState>(des::ConversationKind::DROP_OFF), ctx.currentTime);

    des::EndConversationEvent event(35030, makeConversationSpec(des::ConversationKind::DROP_OFF), false);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getState()->getResult(), des::Result::FAILURE);
    EXPECT_EQ(ctx.tickCount, 1);
}

// --- des::BatteryFullEvent ---

TEST(EventExecute, BatteryFullSetsIdleAndResetsBatteryFlag) {
    MockSimContext ctx;
    ctx.robot->changeState(std::make_unique<des::ChargeState>(), ctx.currentTime);
    ctx.robot->m_batteryFullEventScheduled = true;

    des::BatteryFullEvent event(37000);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getStateType(), des::RobotStateType::IDLE);
    EXPECT_FALSE(ctx.robot->m_batteryFullEventScheduled);
    EXPECT_EQ(ctx.tickCount, 1);
}

// --- des::MissionCompleteEvent ---

TEST(EventExecute, MissionCompleteCallsCompleteOrderAndTicks) {
    MockSimContext ctx;

    auto order = makeAccompanyOrder(1, "Max");
    order->state = des::OrderState::COMPLETED;
    ctx.setOrderPtr(order);

    des::MissionCompleteEvent event(36500, order);
    event.execute(ctx);

    EXPECT_TRUE(ctx.completeOrderCalled);
    EXPECT_EQ(ctx.tickCount, 1);
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

// --- des::StartDriveEvent ---

TEST(EventExecute, StartDriveSetsDrivingAndSchedulesArrival) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    des::StartDriveEvent event(35000, std::make_shared<des::RoomTarget>("Office"));
    event.execute(ctx);

    EXPECT_TRUE(ctx.robot->isDriving());
    EXPECT_EQ(ctx.robot->getTargetLocation(), "Office");
    // Should push a des::StopDriveEvent
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

    des::StartDriveEvent event(35000, std::make_shared<des::RoomTarget>("Office"));
    event.execute(ctx);

    // When already at location, should push des::StopDriveEvent with time=35000
    bool hasImmediateStop = false;
    for (const auto& e : ctx.pushedEvents) {
        if (e->getType() == des::EventType::STOP_DRIVE && e->time == 35000) {
            hasImmediateStop = true;
        }
    }
    EXPECT_TRUE(hasImmediateStop);
}

// --- des::requestDrive (idempotent drive scheduling) ---

TEST(RequestDrive, DuplicateRequestInSameInstantEnqueuesSingleDrive) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    // Two BT ticks at the same instant both request a drive before the first
    // des::StartDriveEvent executes. The second must be a no-op (the root-cause fix).
    des::requestDrive(ctx, "Office");
    des::requestDrive(ctx, "Office");

    int startDrives = 0;
    for (const auto& e : ctx.pushedEvents) {
        if (e->getType() == des::EventType::START_DRIVE) {
            startDrives++;
        }
    }
    EXPECT_EQ(startDrives, 1);
    EXPECT_TRUE(ctx.robot->isDriving());
    EXPECT_EQ(ctx.robot->getTargetLocation(), "Office");
}

TEST(RequestDrive, RequestAfterArrivalEnqueuesAgain) {
    MockSimContext ctx;
    ctx.robot->setDriving(false);

    des::requestDrive(ctx, "Office");
    ctx.robot->setDriving(false);  // StopDrive arrived -> driving cleared
    des::requestDrive(ctx, "Lab");

    int startDrives = 0;
    for (const auto& e : ctx.pushedEvents) {
        if (e->getType() == des::EventType::START_DRIVE) {
            startDrives++;
        }
    }
    EXPECT_EQ(startDrives, 2);
}

// --- des::StopDriveEvent ---

TEST(EventExecute, StopDriveMovesRobotAndSetsDrivingFalse) {
    MockSimContext ctx;
    ctx.robot->setDriving(true);

    des::StopDriveEvent event(35010, std::make_shared<des::RoomTarget>("Office"), 5.0);
    event.execute(ctx);

    EXPECT_EQ(ctx.robot->getLocation(), "Office");
    EXPECT_FALSE(ctx.robot->isDriving());
    EXPECT_EQ(ctx.blackboard["location"], "Office");
    EXPECT_EQ(ctx.tickCount, 1);
}

TEST(EventExecute, StopDriveInAccompanyMovesPerson) {
    MockSimContext ctx;
    ctx.robot->setDriving(true);
    ctx.robot->changeState(std::make_unique<des::AccompanyState>(), ctx.currentTime);

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->roomLabels = {"Office", "MeetingRoom"};
    ctx.employees["Max"] = person.get();
    ctx.personLocations["Max"] = "Office";

    auto order = makeAccompanyOrder(1, "Max", "MeetingRoom");
    ctx.currentOrder = order;

    des::StopDriveEvent event(35100, std::make_shared<des::RoomTarget>("MeetingRoom"), 10.0);
    event.execute(ctx);

    // des::Person should be moved to the robot's arrival location
    EXPECT_EQ(ctx.personLocations["Max"], "MeetingRoom");
}

// --- des::PersonDepartureEvent ---

TEST(EventExecute, PersonDepartureSetsRoomToOutdoor) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->roomLabels = {"Office"};
    ctx.personLocations["Max"] = "Office";

    des::PersonDepartureEvent event(61200, person.get());
    event.execute(ctx);

    EXPECT_EQ(ctx.personLocations["Max"], "OUTDOOR");
    EXPECT_FALSE(ctx.notifiedEvents.empty());
}

// --- des::PersonTransitionEvent ---

TEST(EventExecute, PersonTransitionFromOutdoorDoesNotPushEvent) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->roomLabels = {"Office", "Kitchen"};
    ctx.personLocations["Max"] = "OUTDOOR";

    des::PersonTransitionEvent event(30000, person.get());
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

    des::PersonTransitionEvent event(30000, person.get());
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

    des::PersonTransitionEvent event(30000, person.get());
    event.execute(ctx);

    EXPECT_EQ(ctx.personLocations["Max"], IN_TRANSIT);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_ROOM_ARRIVED);
    EXPECT_GT(ctx.pushedEvents[0]->time, 30000);

    auto arrival = ctx.pushedEvents[0];
    ctx.pushedEvents.clear();
    arrival->execute(ctx);

    EXPECT_EQ(ctx.personLocations["Max"], "IMVS_Kitchen");
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_TRANSITION);
    EXPECT_GT(ctx.pushedEvents[0]->time, arrival->time);
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
    des::Room elevator{"5.2B_Elevator", des::Point{}, 0.0};
    elevator.m_roomType = des::RoomType::ACCESS;
    ctx.rooms.emplace("5.2B_Elevator", elevator);

    des::PersonTransitionEvent event(30000, person.get());
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_ROOM_ARRIVED);
    auto arrival = ctx.pushedEvents[0];
    ctx.pushedEvents.clear();

    arrival->execute(ctx);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    auto leave = ctx.pushedEvents[0];
    EXPECT_EQ(leave->getType(), des::EventType::PERSON_TRANSITION);
    EXPECT_GE(leave->time, arrival->time);
    EXPECT_EQ(ctx.personLocations["Max"], "IMVS_Kitchen");
    ctx.pushedEvents.clear();

    leave->execute(ctx);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    auto atExit = ctx.pushedEvents[0];
    EXPECT_EQ(atExit->getType(), des::EventType::PERSON_ROOM_ARRIVED);
    EXPECT_EQ(ctx.personLocations["Max"], IN_TRANSIT);
    ctx.pushedEvents.clear();

    atExit->execute(ctx);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_DEPARTURE);
    EXPECT_GE(ctx.pushedEvents[0]->time, person->departureTime);
    EXPECT_EQ(ctx.personLocations["Max"], "5.2B_Elevator");
}

TEST(EventExecute, PersonWaitsAtTheExitOnlyForTheAccessStay) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->workplace = "5.2B03";
    person->departureTime = 61200;
    person->roomLabels = {"5.2B03", "5.2B_Elevator"};
    person->transitionMatrix = {
        {0.0, 1.0},
        {1.0, 0.0},
    };
    ctx.personLocations["Max"] = "5.2B03";
    des::Room elevator{"5.2B_Elevator", des::Point{}, 0.0};
    elevator.m_roomType = des::RoomType::ACCESS;
    ctx.rooms.emplace("5.2B_Elevator", elevator);
    des::Room office{"5.2B03", des::Point{}, 0.0};
    office.m_roomType = des::RoomType::OFFICE;
    ctx.rooms.emplace("5.2B03", office);

    des::PersonRoomArrivedEvent arrival(60000, person.get(), "5.2B03");
    arrival.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    auto leave = ctx.pushedEvents[0];
    ctx.pushedEvents.clear();
    leave->execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    auto atExit = ctx.pushedEvents[0];
    ctx.pushedEvents.clear();
    atExit->execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    const int waited = ctx.pushedEvents[0]->time - atExit->time;
    EXPECT_GE(waited, 60);
    EXPECT_LE(waited, 120);
}

TEST(EventExecute, PersonTransitionRetriesWhileBusyInsteadOfEndingChain) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->workplace = "5.2B03";
    person->departureTime = 999999;
    person->busy = true;
    person->roomLabels = {"5.2B03", "IMVS_Kitchen"};
    person->transitionMatrix = {{0.0, 1.0}, {1.0, 0.0}};
    ctx.personLocations["Max"] = "5.2B03";
    des::Room kitchen{"IMVS_Kitchen", des::Point{}, 0.0};
    kitchen.m_roomType = des::RoomType::KITCHEN;
    ctx.rooms.emplace("IMVS_Kitchen", kitchen);
    des::Room office{"5.2B03", des::Point{}, 0.0};
    office.m_roomType = des::RoomType::OFFICE;
    ctx.rooms.emplace("5.2B03", office);

    des::PersonTransitionEvent event(30000, person.get());
    event.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_TRANSITION);
    EXPECT_GT(ctx.pushedEvents[0]->time, 30000);
    EXPECT_EQ(ctx.personLocations["Max"], "5.2B03");
}

TEST(EventExecute, AppointmentEndReleasesPersonWithoutStartingSecondChain) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->busy = true;

    des::AppointmentEndEvent event(30000, person.get());
    event.execute(ctx);

    EXPECT_FALSE(person->busy);
    EXPECT_TRUE(ctx.pushedEvents.empty());
}

TEST(EventExecute, PersonGoesToLunchWhenDueAndResumesAfterwards) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->workplace = "5.2B03";
    person->departureTime = 999999;
    person->lunchTime = 30500;
    person->lunchDuration = 2400;
    person->lunchPending = true;
    person->roomLabels = {"5.2B03", "IMVS_Kitchen"};
    person->transitionMatrix = {{0.0, 1.0}, {1.0, 0.0}};
    ctx.personLocations["Max"] = "5.2B03";
    des::Room kitchen{"IMVS_Kitchen", des::Point{}, 0.0};
    kitchen.m_roomType = des::RoomType::KITCHEN;
    ctx.rooms.emplace("IMVS_Kitchen", kitchen);
    des::Room office{"5.2B03", des::Point{}, 0.0};
    office.m_roomType = des::RoomType::OFFICE;
    ctx.rooms.emplace("5.2B03", office);

    des::PersonRoomArrivedEvent arrival(30000, person.get(), "5.2B03");
    arrival.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    auto lunch = ctx.pushedEvents[0];
    EXPECT_EQ(lunch->time, person->lunchTime);
    EXPECT_FALSE(person->lunchPending);

    ctx.pushedEvents.clear();
    lunch->execute(ctx);

    EXPECT_EQ(ctx.personLocations["Max"], IN_TRANSIT);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_ROOM_ARRIVED);
    auto lunchArrival = ctx.pushedEvents[0];
    EXPECT_GE(lunchArrival->time, person->lunchTime);

    ctx.pushedEvents.clear();
    lunchArrival->execute(ctx);

    EXPECT_EQ(ctx.personLocations["Max"], "IMVS_Kitchen");
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_TRANSITION);
    EXPECT_EQ(ctx.pushedEvents[0]->time, lunchArrival->time + 2400);
}

TEST(EventExecute, FailedAccompanyReleasesPersonWithoutAppointmentEnd) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->busy = true;
    ctx.employees["Max"] = person.get();

    auto order = std::make_shared<des::AccompanyOrder>();
    order->personName = "Max";
    order->state = des::FAILED;

    des::AccompanyOrderPlugin plugin;
    plugin.onMissionEnd(ctx, *order);

    EXPECT_FALSE(person->busy);
    EXPECT_TRUE(ctx.pushedEvents.empty());
}

TEST(EventExecute, CompletedAccompanySchedulesAppointmentEnd) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->busy = true;
    ctx.employees["Max"] = person.get();

    auto order = std::make_shared<des::AccompanyOrder>();
    order->personName = "Max";
    order->state = des::COMPLETED;

    des::AccompanyOrderPlugin plugin;
    plugin.onMissionEnd(ctx, *order);

    EXPECT_TRUE(person->busy);
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::APPOINTMENT_END);
}

TEST(EventExecute, PersonWithoutSampledLunchDoesNotGoToLunch) {
    MockSimContext ctx;

    auto person = std::make_shared<des::Person>();
    person->firstName = "Max";
    person->workplace = "5.2B03";
    person->departureTime = 999999;
    person->roomLabels = {"5.2B03", "IMVS_Kitchen"};
    person->transitionMatrix = {{0.0, 1.0}, {1.0, 0.0}};
    ctx.personLocations["Max"] = "5.2B03";
    des::Room kitchen{"IMVS_Kitchen", des::Point{}, 0.0};
    kitchen.m_roomType = des::RoomType::KITCHEN;
    ctx.rooms.emplace("IMVS_Kitchen", kitchen);
    des::Room office{"5.2B03", des::Point{}, 0.0};
    office.m_roomType = des::RoomType::OFFICE;
    ctx.rooms.emplace("5.2B03", office);

    des::PersonRoomArrivedEvent arrival(30000, person.get(), "5.2B03");
    arrival.execute(ctx);

    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_TRANSITION);
}

// --- des::PersonArrivedEvent ---

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

    des::PersonArrivedEvent event(30000, person.get());
    event.execute(ctx);

    // Should transition to a new room via the matrix
    EXPECT_EQ(ctx.personLocations["Max"], "IMVS_Kitchen");
    // Should push a des::PersonTransitionEvent with short delay (10-30s)
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

    des::PersonArrivedEvent event(30000, person.get());
    event.execute(ctx);

    EXPECT_EQ(ctx.personLocations["Max"], "Office");
    ASSERT_EQ(ctx.pushedEvents.size(), 1u);
    EXPECT_EQ(ctx.pushedEvents[0]->getType(), des::EventType::PERSON_TRANSITION);
}

// --- Event metadata ---

TEST(EventMetadata, EventTypesAreCorrect) {
    EXPECT_EQ(des::SimulationStartEvent(0).getType(), des::EventType::SIMULATION_START);
    EXPECT_EQ(des::SimulationEndEvent(0).getType(), des::EventType::SIMULATION_END);
    EXPECT_EQ(des::AbortSearchEvent(0, nullptr).getType(), des::EventType::ABORT_SEARCH);
    EXPECT_EQ(des::BatteryFullEvent(0).getType(), des::EventType::BATTERY_FULL);
    EXPECT_EQ(des::StartAccompanyEvent(0, nullptr).getType(), des::EventType::START_ACCOMPANY);
    EXPECT_EQ(des::StartConversationEvent(0, makeConversationSpec(des::ConversationKind::DROP_OFF)).getType(), des::EventType::CONVERSATION_START);
    EXPECT_EQ(des::EndConversationEvent(0, makeConversationSpec(des::ConversationKind::DROP_OFF), true).getType(), des::EventType::CONVERSATION_END);
    EXPECT_EQ(des::StopDriveEvent(0, std::make_shared<des::RoomTarget>("x"), 0).getType(), des::EventType::STOP_DRIVE);
    EXPECT_EQ(des::StartDriveEvent(0, std::make_shared<des::RoomTarget>("x")).getType(), des::EventType::START_DRIVE);
}

TEST(EventMetadata, EventNamesAreNonEmpty) {
    EXPECT_FALSE(des::SimulationStartEvent(0).getName().empty());
    EXPECT_FALSE(des::SimulationEndEvent(0).getName().empty());
    EXPECT_FALSE(des::AbortSearchEvent(0, nullptr).getName().empty());
    EXPECT_FALSE(des::BatteryFullEvent(0).getName().empty());
    EXPECT_FALSE(des::StopDriveEvent(0, std::make_shared<des::RoomTarget>("X"), 0).getName().empty());
    EXPECT_FALSE(des::StartDriveEvent(0, std::make_shared<des::RoomTarget>("X")).getName().empty());
}
