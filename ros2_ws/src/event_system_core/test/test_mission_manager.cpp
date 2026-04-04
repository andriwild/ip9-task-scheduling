#include <gtest/gtest.h>
#include <memory>

#include "../src/model/mission_manager.h"

class MissionManagerTest : public ::testing::Test {
protected:
    MissionManager manager;

    static std::shared_ptr<des::Appointment> makeAppointment(int id, const std::string& person) {
        auto appt = std::make_shared<des::Appointment>();
        appt->id = id;
        appt->personName = person;
        appt->roomName = "Room" + std::to_string(id);
        appt->appointmentTime = 1000 * id;
        appt->description = "Test " + std::to_string(id);
        return appt;
    }
};

TEST_F(MissionManagerTest, InitialStateHasNoCurrent) {
    EXPECT_EQ(manager.getCurrent(), nullptr);
}

TEST_F(MissionManagerTest, InitialStateHasNoPending) {
    EXPECT_FALSE(manager.hasPending());
    EXPECT_EQ(manager.peekPending(), nullptr);
    EXPECT_EQ(manager.popPending(), nullptr);
}

TEST_F(MissionManagerTest, SetAndGetCurrent) {
    auto appt = makeAppointment(1, "Max");
    manager.setCurrent(appt);
    EXPECT_EQ(manager.getCurrent(), appt);
    EXPECT_EQ(manager.getCurrent()->personName, "Max");
}

TEST_F(MissionManagerTest, OverwriteCurrent) {
    manager.setCurrent(makeAppointment(1, "Max"));
    auto second = makeAppointment(2, "Anna");
    manager.setCurrent(second);
    EXPECT_EQ(manager.getCurrent()->personName, "Anna");
}

TEST_F(MissionManagerTest, UpdateStateOnCurrent) {
    auto appt = makeAppointment(1, "Max");
    manager.setCurrent(appt);

    manager.updateState(des::MissionState::IN_PROGRESS);
    EXPECT_EQ(manager.getCurrent()->state, des::MissionState::IN_PROGRESS);

    manager.updateState(des::MissionState::COMPLETED);
    EXPECT_EQ(manager.getCurrent()->state, des::MissionState::COMPLETED);
}

TEST_F(MissionManagerTest, AddAndPopPending) {
    auto a1 = makeAppointment(1, "Max");
    auto a2 = makeAppointment(2, "Anna");

    manager.addPending(a1);
    manager.addPending(a2);

    EXPECT_TRUE(manager.hasPending());
    EXPECT_EQ(manager.peekPending(), a1);

    auto popped = manager.popPending();
    EXPECT_EQ(popped, a1);
    EXPECT_EQ(manager.peekPending(), a2);

    popped = manager.popPending();
    EXPECT_EQ(popped, a2);
    EXPECT_FALSE(manager.hasPending());
}

TEST_F(MissionManagerTest, PendingIsFIFO) {
    for (int i = 0; i < 5; ++i) {
        manager.addPending(makeAppointment(i, "Person" + std::to_string(i)));
    }

    for (int i = 0; i < 5; ++i) {
        auto popped = manager.popPending();
        EXPECT_EQ(popped->id, i);
    }
}

TEST_F(MissionManagerTest, PeekDoesNotRemove) {
    manager.addPending(makeAppointment(1, "Max"));

    auto peeked1 = manager.peekPending();
    auto peeked2 = manager.peekPending();
    EXPECT_EQ(peeked1, peeked2);
    EXPECT_TRUE(manager.hasPending());
}

TEST_F(MissionManagerTest, ResetClearsEverything) {
    manager.setCurrent(makeAppointment(1, "Max"));
    manager.addPending(makeAppointment(2, "Anna"));
    manager.addPending(makeAppointment(3, "Leo"));

    manager.reset();

    EXPECT_EQ(manager.getCurrent(), nullptr);
    EXPECT_FALSE(manager.hasPending());
    EXPECT_EQ(manager.popPending(), nullptr);
}

TEST_F(MissionManagerTest, ResetThenReuseWorks) {
    manager.setCurrent(makeAppointment(1, "Max"));
    manager.reset();

    auto appt = makeAppointment(99, "NewPerson");
    manager.setCurrent(appt);
    EXPECT_EQ(manager.getCurrent()->id, 99);

    manager.addPending(makeAppointment(100, "AnotherPerson"));
    EXPECT_TRUE(manager.hasPending());
}
