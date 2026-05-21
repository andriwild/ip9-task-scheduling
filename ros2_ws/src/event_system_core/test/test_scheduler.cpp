#include <gtest/gtest.h>
#include <memory>
#include <optional>

#include "../src/sim/i_path_planner.h"
#include "../src/sim/scheduler.h"
#include "../src/plugins/order_registry.h"
#include "../src/plugins/accompany/accompany_plugin.h"
#include "../src/plugins/accompany/accompany_order.h"

class MockPathPlanner : public IPathPlanner {
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

// Set the (singleton) AccompanyOrderPlugin's config — accompany_speed and
// related params live on the plugin now, not on SimConfig.
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
    OrderRegistry::instance().get(AccompanyOrderPlugin::kTypeName).loadConfig(j);
}

class SchedulerTest : public ::testing::Test {
protected:
    std::shared_ptr<MockPathPlanner> planner;
    std::shared_ptr<des::SimConfig> config;
    des::PersonLocationMap locations;
    des::SearchAreaMap searchAreas;

    static void SetUpTestSuite() {
        static bool registered = false;
        if (!registered) {
            OrderRegistry::instance().registerPlugin(std::make_unique<AccompanyOrderPlugin>());
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

        auto max = std::make_shared<des::Person>();
        max->firstName = "Max";
        max->roomLabels = {"Office"};
        locations["Max"] = max;

        auto anna = std::make_shared<des::Person>();
        anna->firstName = "Anna";
        anna->roomLabels = {"Lab"};
        locations["Anna"] = anna;

        // scanTime is part of the accompany plugin's pessimistic meeting calc; keep zero for clean drive-time assertions.
        searchAreas["Office"] = 0.0;
        searchAreas["Lab"] = 0.0;

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

    std::unique_ptr<Scheduler> makeScheduler() {
        return std::make_unique<Scheduler>(config, planner, locations, searchAreas);
    }

    static std::shared_ptr<AccompanyOrder> makeAccompanyOrder(
            int id,
            const std::string& person,
            const std::string& room,
            int appointmentTime,
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
};

// --- simplePlan (exercises the plugin's pessimistic-meeting math) ---

TEST_F(SchedulerTest, SimplePlanCalculatesCorrectStartTimes) {
    auto scheduler = makeScheduler();

    des::OrderList orders = { makeAccompanyOrder(1, "Max", "MeetingRoom", 36000, "Test Meeting") };

    auto missions = scheduler->simplePlan(orders, "Dock");
    ASSERT_EQ(missions.size(), 1u);

    // pessimisticMeeting for Max with single location ["Office"]:
    //   search Dock->Office (10/1.0 = 10)
    //   accompany Office->MeetingRoom (20/0.5 = 40)
    // total = 50. timeBuffer = 60. startTime = 36000 - 50 - 60 = 35890
    EXPECT_EQ(missions[0]->time, 35890);
    auto accompany = std::dynamic_pointer_cast<AccompanyOrder>(missions[0]->orderPtr);
    ASSERT_NE(accompany, nullptr);
    EXPECT_EQ(accompany->personName, "Max");
}

TEST_F(SchedulerTest, SimplePlanMultipleAppointments) {
    auto scheduler = makeScheduler();

    des::OrderList orders = {
        makeAccompanyOrder(1, "Max",  "MeetingRoom", 36000, "Meeting 1"),
        makeAccompanyOrder(2, "Anna", "HallA",       39600, "Meeting 2"),
    };

    auto missions = scheduler->simplePlan(orders, "Dock");
    ASSERT_EQ(missions.size(), 2u);

    EXPECT_EQ(missions[0]->time, 35890);
    // Anna: pessimistic = 12 + 16 = 28s, start = 39600 - 28 - 60 = 39512
    EXPECT_EQ(missions[1]->time, 39512);
}

TEST_F(SchedulerTest, SimplePlanWithZeroTimeBuffer) {
    config->timeBuffer = 0.0;
    auto scheduler = makeScheduler();

    des::OrderList orders = { makeAccompanyOrder(1, "Max", "MeetingRoom", 36000) };

    auto missions = scheduler->simplePlan(orders, "Dock");
    ASSERT_EQ(missions.size(), 1u);

    // Without buffer: 36000 - 50 - 0 = 35950
    EXPECT_EQ(missions[0]->time, 35950);
}

TEST_F(SchedulerTest, SimplePlanEmptyAppointments) {
    auto scheduler = makeScheduler();

    des::OrderList orders;
    auto missions = scheduler->simplePlan(orders, "Dock");
    EXPECT_TRUE(missions.empty());
}

TEST_F(SchedulerTest, PessimisticMeetingWithMultipleSearchLocations) {
    // Give Anna multiple rooms — pessimistic search visits each in order.
    locations["Anna"]->roomLabels = {"Lab", "Kitchen"};
    planner->setDistance("Lab", "Kitchen", 7.0);
    auto scheduler = makeScheduler();

    // Anna has rooms ["Lab", "Kitchen"]
    //   search Dock->Lab (12/1.0) = 12
    //   search Lab->Kitchen (7/1.0) = 7
    //   accompany Kitchen->HallA (18/0.5) = 36
    // total = 55. startTime = 39600 - 55 - 60 = 39485
    des::OrderList orders = { makeAccompanyOrder(1, "Anna", "HallA", 39600) };
    auto missions = scheduler->simplePlan(orders, "Dock");
    ASSERT_EQ(missions.size(), 1u);
    EXPECT_EQ(missions[0]->time, 39485);
}

// --- Speed impact ---

TEST_F(SchedulerTest, HigherSpeedReducesDriveTime) {
    config->robotSpeed = 2.0;        // double the base drive speed
    setAccompanyConfig(/*accompanySpeed=*/1.0);
    auto scheduler = makeScheduler();

    // Max: pessimistic = Dock->Office (10/2.0 = 5) + Office->MeetingRoom (20/1.0 = 20) = 25
    // startTime = 36000 - 25 - 60 = 35915
    des::OrderList orders = { makeAccompanyOrder(1, "Max", "MeetingRoom", 36000) };
    auto missions = scheduler->simplePlan(orders, "Dock");
    ASSERT_EQ(missions.size(), 1u);
    EXPECT_EQ(missions[0]->time, 35915);
}
