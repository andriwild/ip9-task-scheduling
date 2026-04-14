#include <gtest/gtest.h>
#include <memory>

#include "../src/model/mission_manager.h"
#include "../src/plugins/accompany/accompany_order.h"

class MissionManagerTest : public ::testing::Test {
protected:
    MissionManager manager;

    static std::shared_ptr<AccompanyOrder> makeOrder(int id, const std::string& person) {
        auto order = std::make_shared<AccompanyOrder>();
        order->id = id;
        order->type = "accompany";
        order->personName = person;
        order->roomName = "Room" + std::to_string(id);
        order->deadline = 1000 * id;
        order->description = "Test " + std::to_string(id);
        return order;
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
    auto order = makeOrder(1, "Max");
    manager.setCurrent(order);
    EXPECT_EQ(manager.getCurrent(), order);
    EXPECT_EQ(std::dynamic_pointer_cast<AccompanyOrder>(manager.getCurrent())->personName, "Max");
}

TEST_F(MissionManagerTest, OverwriteCurrent) {
    manager.setCurrent(makeOrder(1, "Max"));
    auto second = makeOrder(2, "Anna");
    manager.setCurrent(second);
    EXPECT_EQ(std::dynamic_pointer_cast<AccompanyOrder>(manager.getCurrent())->personName, "Anna");
}

TEST_F(MissionManagerTest, UpdateStateOnCurrent) {
    auto order = makeOrder(1, "Max");
    manager.setCurrent(order);

    manager.updateState(des::MissionState::IN_PROGRESS);
    EXPECT_EQ(manager.getCurrent()->state, des::MissionState::IN_PROGRESS);

    manager.updateState(des::MissionState::COMPLETED);
    EXPECT_EQ(manager.getCurrent()->state, des::MissionState::COMPLETED);
}

TEST_F(MissionManagerTest, AddAndPopPending) {
    auto a1 = makeOrder(1, "Max");
    auto a2 = makeOrder(2, "Anna");

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
        manager.addPending(makeOrder(i, "Person" + std::to_string(i)));
    }

    for (int i = 0; i < 5; ++i) {
        auto popped = manager.popPending();
        EXPECT_EQ(popped->id, i);
    }
}

TEST_F(MissionManagerTest, PeekDoesNotRemove) {
    manager.addPending(makeOrder(1, "Max"));

    auto peeked1 = manager.peekPending();
    auto peeked2 = manager.peekPending();
    EXPECT_EQ(peeked1, peeked2);
    EXPECT_TRUE(manager.hasPending());
}

TEST_F(MissionManagerTest, ResetClearsEverything) {
    manager.setCurrent(makeOrder(1, "Max"));
    manager.addPending(makeOrder(2, "Anna"));
    manager.addPending(makeOrder(3, "Leo"));

    manager.reset();

    EXPECT_EQ(manager.getCurrent(), nullptr);
    EXPECT_FALSE(manager.hasPending());
    EXPECT_EQ(manager.popPending(), nullptr);
}

TEST_F(MissionManagerTest, ResetThenReuseWorks) {
    manager.setCurrent(makeOrder(1, "Max"));
    manager.reset();

    auto order = makeOrder(99, "NewPerson");
    manager.setCurrent(order);
    EXPECT_EQ(manager.getCurrent()->id, 99);

    manager.addPending(makeOrder(100, "AnotherPerson"));
    EXPECT_TRUE(manager.hasPending());
}
