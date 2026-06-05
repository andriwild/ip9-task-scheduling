#include <gtest/gtest.h>
#include <memory>

#include "../src/model/mission/scheduled_mission_queue.h"
#include "../src/model/mission/interrupt_mission_slot.h"
#include "../src/plugins/accompany/accompany_order.h"

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

// --- ScheduledMissionQueue ---

class ScheduledMissionQueueTest : public ::testing::Test {
protected:
    ScheduledMissionQueue queue;
};

TEST_F(ScheduledMissionQueueTest, InitialStateIsEmpty) {
    EXPECT_FALSE(queue.has());
    EXPECT_EQ(queue.peek(), nullptr);
    EXPECT_EQ(queue.pop(), nullptr);
    EXPECT_EQ(queue.size(), 0u);
}

TEST_F(ScheduledMissionQueueTest, AddAndPop) {
    auto a1 = makeOrder(1, "Max");
    auto a2 = makeOrder(2, "Anna");

    queue.add(a1);
    queue.add(a2);

    EXPECT_TRUE(queue.has());
    EXPECT_EQ(queue.size(), 2u);
    EXPECT_EQ(queue.peek(), a1);

    auto popped = queue.pop();
    EXPECT_EQ(popped, a1);
    EXPECT_EQ(queue.peek(), a2);

    popped = queue.pop();
    EXPECT_EQ(popped, a2);
    EXPECT_FALSE(queue.has());
}

TEST_F(ScheduledMissionQueueTest, IsFIFO) {
    for (int i = 0; i < 5; ++i) {
        queue.add(makeOrder(i, "Person" + std::to_string(i)));
    }

    for (int i = 0; i < 5; ++i) {
        auto popped = queue.pop();
        EXPECT_EQ(popped->id, i);
    }
}

TEST_F(ScheduledMissionQueueTest, PeekDoesNotRemove) {
    queue.add(makeOrder(1, "Max"));

    auto peeked1 = queue.peek();
    auto peeked2 = queue.peek();
    EXPECT_EQ(peeked1, peeked2);
    EXPECT_TRUE(queue.has());
}

TEST_F(ScheduledMissionQueueTest, ClearEmptiesQueue) {
    queue.add(makeOrder(1, "Max"));
    queue.add(makeOrder(2, "Anna"));

    queue.clear();

    EXPECT_FALSE(queue.has());
    EXPECT_EQ(queue.size(), 0u);
    EXPECT_EQ(queue.pop(), nullptr);
}

// --- InterruptMissionSlot ---

class InterruptMissionSlotTest : public ::testing::Test {
protected:
    InterruptMissionSlot slot;
};

TEST_F(InterruptMissionSlotTest, InitialStateIsEmpty) {
    EXPECT_FALSE(slot.has());
    EXPECT_EQ(slot.active(), nullptr);
}

TEST_F(InterruptMissionSlotTest, PushStoresInterrupt) {
    auto current   = makeOrder(1, "Max");
    auto interrupt = makeOrder(99, "Info");

    EXPECT_TRUE(slot.push(interrupt, current));
    EXPECT_TRUE(slot.has());
    EXPECT_EQ(slot.active(), interrupt);
}

TEST_F(InterruptMissionSlotTest, PushWithoutCurrentIsAllowed) {
    auto interrupt = makeOrder(99, "Info");
    EXPECT_TRUE(slot.push(interrupt, nullptr));
    EXPECT_EQ(slot.active(), interrupt);
}

TEST_F(InterruptMissionSlotTest, PopClearsActiveInterrupt) {
    auto interrupt = makeOrder(99, "Info");
    slot.push(interrupt, nullptr);

    slot.pop(interrupt);
    EXPECT_FALSE(slot.has());
    EXPECT_EQ(slot.active(), nullptr);
}

TEST_F(InterruptMissionSlotTest, PushRejectedWhenAlreadyActive) {
    auto first  = makeOrder(91, "Info1");
    auto second = makeOrder(92, "Info2");

    EXPECT_TRUE(slot.push(first, nullptr));
    EXPECT_FALSE(slot.push(second, nullptr));
    EXPECT_EQ(slot.active(), first);
}

TEST_F(InterruptMissionSlotTest, ClearRemovesInterrupt) {
    slot.push(makeOrder(99, "Info"), nullptr);
    slot.clear();
    EXPECT_FALSE(slot.has());
}
