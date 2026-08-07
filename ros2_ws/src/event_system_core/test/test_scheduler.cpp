#include <gtest/gtest.h>
#include <memory>
#include <optional>

#include "../src/sim/i_path_planner.h"
#include "../src/sim/scheduler.h"
#include "../src/plugins/order_registry.h"
#include "../src/plugins/accompany/accompany_plugin.h"
#include "../src/plugins/accompany/accompany_order.h"

class MockPathPlanner : public des::IPathPlanner {
    std::map<std::pair<std::string, std::string>, double> m_distances;

public:
    void setDistance(const std::string& from, const std::string& to, double distance) {
        m_distances[{from, to}] = distance;
    }

    std::optional<double> calcDistance(const std::string& from, const std::string& to, bool /*useCache*/) override {
        auto it = m_distances.find({from, to});
        if (it != m_distances.end()) {
            return it->second;
        }
        return std::nullopt;
    }
};

// Set the (singleton) des::AccompanyOrderPlugin's config — accompany_speed and
// related params live on the plugin now, not on des::SimConfig.
static void setAccompanyConfig(double accompanySpeed,
                               double conversationProbability = 0.5,
                               double conversationDurationMean = 30.0,
                               double conversationDurationStd  = 0.0,
                               double appointmentDuration      = 1800.0) {
    nlohmann::json j = {
        {"accompany_speed",            accompanySpeed},
        {"conversation_probability",   conversationProbability},
        {"conversation_duration_mean", conversationDurationMean},
        {"conversation_duration_std",  conversationDurationStd},
        {"appointment_duration",       appointmentDuration},
    };
    des::OrderRegistry::instance().get(des::AccompanyOrderPlugin::kTypeName).loadConfig(j);
}

class SchedulerTest : public ::testing::Test {
protected:
    std::shared_ptr<MockPathPlanner> planner;
    std::shared_ptr<des::SimConfig> config;
    des::PersonMap employees;
    des::PersonList ownedPeople;
    des::RoomMap locationMap;

    static void SetUpTestSuite() {
        static bool registered = false;
        if (!registered) {
            des::OrderRegistry::instance().registerPlugin(std::make_unique<des::AccompanyOrderPlugin>());
            registered = true;
        }
    }

    void SetUp() override {
        planner = std::make_shared<MockPathPlanner>();

        config = std::make_shared<des::SimConfig>();
        config->robotSpeed = 1.0;
        config->timeBuffer = 60.0;
        config->cacheEnabled = false;

        // Accompany-specific params now live on the plugin.
        setAccompanyConfig(/*accompanySpeed=*/0.5);

        auto max = std::make_unique<des::Person>();
        max->firstName = "Max";
        max->roomLabels = {"Office"};
        employees["Max"] = max.get();
        ownedPeople.push_back(std::move(max));

        auto anna = std::make_unique<des::Person>();
        anna->firstName = "Anna";
        anna->roomLabels = {"Lab"};
        employees["Anna"] = anna.get();
        ownedPeople.push_back(std::move(anna));

        // scanTime is part of the accompany plugin's pessimistic meeting calc; keep zero for clean drive-time assertions.
        locationMap.emplace("Office", des::Room("Office", {}, 0.0));
        locationMap.emplace("Lab", des::Room("Lab", {}, 0.0));
        locationMap.emplace("Kitchen", des::Room("Kitchen", {}, 0.0));

        // Dock -> Office = 10m, Office -> MeetingRoom = 20m
        planner->setDistance("Dock", "Office", 10.0);
        planner->setDistance("Office", "MeetingRoom", 20.0);

        // Dock -> Kitchen = 15m, Kitchen -> Lab = 5m, Lab -> HallA = 8m
        planner->setDistance("Dock", "Kitchen", 15.0);
        planner->setDistance("Kitchen", "Lab", 5.0);
        planner->setDistance("Dock", "Lab", 12.0);
        planner->setDistance("Lab", "HallA", 8.0);
        planner->setDistance("Kitchen", "HallA", 18.0);
    }

    std::unique_ptr<des::Scheduler> makeScheduler() {
        return std::make_unique<des::Scheduler>(config, planner, locationMap);
    }

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
};

// --- simplePlan (exercises the plugin's pessimistic-meeting math) ---

TEST_F(SchedulerTest, SimplePlanCalculatesCorrectStartTimes) {
    auto scheduler = makeScheduler();

    des::OrderList orders = { makeAccompanyOrder(1, "Max", "MeetingRoom", 36000, "Test Meeting") };

    auto missions = scheduler->createMissionDispatchEvents(orders, "Dock");
    ASSERT_EQ(missions.size(), 1u);

    EXPECT_EQ(missions[0]->time, 35940);
    auto accompany = std::dynamic_pointer_cast<des::AccompanyOrder>(missions[0]->orderPtr);
    ASSERT_NE(accompany, nullptr);
    EXPECT_EQ(accompany->personName, "Max");
}

TEST_F(SchedulerTest, SimplePlanMultipleAppointments) {
    auto scheduler = makeScheduler();

    des::OrderList orders = {
        makeAccompanyOrder(1, "Max",  "MeetingRoom", 36000, "Meeting 1"),
        makeAccompanyOrder(2, "Anna", "HallA",       39600, "Meeting 2"),
    };

    auto missions = scheduler->createMissionDispatchEvents(orders, "Dock");
    ASSERT_EQ(missions.size(), 2u);

    EXPECT_EQ(missions[0]->time, 35940);
    EXPECT_EQ(missions[1]->time, 39540);
}

TEST_F(SchedulerTest, SimplePlanWithZeroTimeBuffer) {
    config->timeBuffer = 0.0;
    auto scheduler = makeScheduler();

    des::OrderList orders = { makeAccompanyOrder(1, "Max", "MeetingRoom", 36000) };

    auto missions = scheduler->createMissionDispatchEvents(orders, "Dock");
    ASSERT_EQ(missions.size(), 1u);

    EXPECT_EQ(missions[0]->time, 36000);
}

TEST_F(SchedulerTest, SimplePlanEmptyAppointments) {
    auto scheduler = makeScheduler();

    des::OrderList orders;
    auto missions = scheduler->createMissionDispatchEvents(orders, "Dock");
    EXPECT_TRUE(missions.empty());
}

TEST_F(SchedulerTest, DispatchIndependentOfPersonRooms) {
    employees["Anna"]->roomLabels = {"Lab", "Kitchen"};
    planner->setDistance("Lab", "Kitchen", 7.0);
    auto scheduler = makeScheduler();

    des::OrderList orders = { makeAccompanyOrder(1, "Anna", "HallA", 39600) };
    auto missions = scheduler->createMissionDispatchEvents(orders, "Dock");
    ASSERT_EQ(missions.size(), 1u);
    EXPECT_EQ(missions[0]->time, 39540);
}
