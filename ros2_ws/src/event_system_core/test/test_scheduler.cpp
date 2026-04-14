#include <gtest/gtest.h>
#include <memory>
#include <optional>

#include "../src/sim/i_path_planner.h"
#include "../src/sim/scheduler.h"

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

class SchedulerTest : public ::testing::Test {
protected:
    std::shared_ptr<MockPathPlanner> planner;
    std::shared_ptr<des::SimConfig> config;
    des::PersonLocationMap locations;
    des::SearchAreaMap searchAreas;

    void SetUp() override {
        planner = std::make_shared<MockPathPlanner>();

        config = std::make_shared<des::SimConfig>();
        config->robotSpeed = 1.0;
        config->robotAccompanySpeed = 0.5;
        config->timeBuffer = 60.0;
        config->cacheEnabled = false;

        auto max = std::make_shared<des::Person>();
        max->firstName = "Max";
        max->roomLabels = {"Office", "Kitchen", "Elevator"};
        locations["Max"] = max;

        auto anna = std::make_shared<des::Person>();
        anna->firstName = "Anna";
        anna->roomLabels = {"Lab", "Kitchen"};
        locations["Anna"] = anna;

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
};

// --- optimisticMeeting ---

TEST_F(SchedulerTest, OptimisticMeetingUsesFirstLocation) {
    auto scheduler = makeScheduler();

    // Max's first room is "Office". Dock->Office at speed 1.0 = 10s, Office->MeetingRoom at speed 0.5 = 40s
    double result = scheduler->optimisticMeeting("Max", "Dock", "MeetingRoom");
    EXPECT_DOUBLE_EQ(result, 10.0 / 1.0 + 20.0 / 0.5); // 10 + 40 = 50
}

TEST_F(SchedulerTest, OptimisticMeetingWithDifferentPerson) {
    auto scheduler = makeScheduler();

    // Anna's first room is "Lab". Dock->Lab at speed 1.0 = 12s, Lab->HallA at speed 0.5 = 16s
    double result = scheduler->optimisticMeeting("Anna", "Dock", "HallA");
    EXPECT_DOUBLE_EQ(result, 12.0 / 1.0 + 8.0 / 0.5); // 12 + 16 = 28
}

// --- pessimisticMeeting ---

TEST_F(SchedulerTest, PessimisticMeetingSearchesAllLocations) {
    auto scheduler = makeScheduler();

    // Anna has rooms ["Lab", "Kitchen"]
    // Dock->Lab at speed 1.0 = 12s, Lab->Kitchen at speed 1.0 = ?
    planner->setDistance("Lab", "Kitchen", 7.0);

    // searchTime = Dock->Lab (12/1.0) + Lab->Kitchen (7/1.0) = 19
    // accompanyTime = Kitchen->HallA (18/0.5) = 36
    double result = scheduler->pessimisticMeeting("Anna", "Dock", "HallA");
    EXPECT_DOUBLE_EQ(result, 12.0 + 7.0 + 18.0 / 0.5); // 19 + 36 = 55
}

// --- simplePlan ---

TEST_F(SchedulerTest, SimplePlanCalculatesCorrectStartTimes) {
    auto scheduler = makeScheduler();

    auto appt = std::make_shared<des::Appointment>();
    appt->personName = "Max";
    appt->roomName = "MeetingRoom";
    appt->appointmentTime = 36000; // 10:00:00
    appt->description = "Test Meeting";

    des::AppointmentList appointments = {appt};

    auto missions = scheduler->simplePlan(appointments, "Dock");
    ASSERT_EQ(missions.size(), 1u);

    // optimisticMeeting = 50s, timeBuffer = 60s
    // startTime = 36000 - 50 - 60 = 35890
    EXPECT_EQ(missions[0]->time, 35890);
    EXPECT_EQ(missions[0]->appointment->personName, "Max");
}

TEST_F(SchedulerTest, SimplePlanMultipleAppointments) {
    auto scheduler = makeScheduler();

    auto appt1 = std::make_shared<des::Appointment>();
    appt1->personName = "Max";
    appt1->roomName = "MeetingRoom";
    appt1->appointmentTime = 36000;
    appt1->description = "Meeting 1";

    auto appt2 = std::make_shared<des::Appointment>();
    appt2->personName = "Anna";
    appt2->roomName = "HallA";
    appt2->appointmentTime = 39600;
    appt2->description = "Meeting 2";

    des::AppointmentList appointments = {appt1, appt2};

    auto missions = scheduler->simplePlan(appointments, "Dock");
    ASSERT_EQ(missions.size(), 2u);

    // Max: optimistic = 50s, start = 36000 - 50 - 60 = 35890
    EXPECT_EQ(missions[0]->time, 35890);

    // Anna: optimistic = 12 + 16 = 28s, start = 39600 - 28 - 60 = 39512
    EXPECT_EQ(missions[1]->time, 39512);
}

TEST_F(SchedulerTest, SimplePlanWithZeroTimeBuffer) {
    config->timeBuffer = 0.0;
    auto scheduler = makeScheduler();

    auto appt = std::make_shared<des::Appointment>();
    appt->personName = "Max";
    appt->roomName = "MeetingRoom";
    appt->appointmentTime = 36000;
    appt->description = "Test";

    des::AppointmentList appointments = {appt};

    auto missions = scheduler->simplePlan(appointments, "Dock");
    ASSERT_EQ(missions.size(), 1u);

    // Without buffer: 36000 - 50 - 0 = 35950
    EXPECT_EQ(missions[0]->time, 35950);
}

TEST_F(SchedulerTest, SimplePlanEmptyAppointments) {
    auto scheduler = makeScheduler();

    des::AppointmentList appointments;
    auto missions = scheduler->simplePlan(appointments, "Dock");
    EXPECT_TRUE(missions.empty());
}

// --- Speed impact ---

TEST_F(SchedulerTest, HigherSpeedReducesDriveTime) {
    config->robotSpeed = 2.0;        // double speed
    config->robotAccompanySpeed = 1.0;
    auto scheduler = makeScheduler();

    // Max: Dock->Office = 10/2.0 = 5s, Office->MeetingRoom = 20/1.0 = 20s
    double result = scheduler->optimisticMeeting("Max", "Dock", "MeetingRoom");
    EXPECT_DOUBLE_EQ(result, 5.0 + 20.0); // 25
}
