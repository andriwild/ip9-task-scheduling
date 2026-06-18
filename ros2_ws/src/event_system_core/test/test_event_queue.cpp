#include <gtest/gtest.h>
#include <memory>

#include "../src/model/event.h"
#include "../src/model/event_queue.h"

// Minimal concrete event for testing (no ROS dependency)
class DummyEvent final : public IEvent {
    des::EventType m_type;
public:
    explicit DummyEvent(int time, des::EventType type = des::EventType::SIMULATION_START)
        : IEvent(time), m_type(type) {}
    void execute(ISimContext&) override {}
    std::string getName() const override { return "Dummy"; }
    des::EventType getType() const override { return m_type; }
    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<DummyEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }
};

class EventQueueTest : public ::testing::Test {
protected:
    EventQueue queue;

    static std::shared_ptr<IEvent> makeEvent(int time) {
        return std::make_shared<DummyEvent>(time);
    }

    static std::shared_ptr<IEvent> makeEvent(int time, des::EventType type) {
        return std::make_shared<DummyEvent>(time, type);
    }
};

TEST_F(EventQueueTest, NewQueueIsEmpty) {
    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), 0u);
    EXPECT_EQ(queue.top(), nullptr);
}

TEST_F(EventQueueTest, PushSingleEvent) {
    queue.push(makeEvent(100));
    EXPECT_FALSE(queue.empty());
    EXPECT_EQ(queue.size(), 1u);
    EXPECT_EQ(queue.top()->time, 100);
}

TEST_F(EventQueueTest, PopReturnsEventsInChronologicalOrder) {
    queue.push(makeEvent(300));
    queue.push(makeEvent(100));
    queue.push(makeEvent(200));

    EXPECT_EQ(queue.top()->time, 100);
    queue.pop();
    EXPECT_EQ(queue.top()->time, 200);
    queue.pop();
    EXPECT_EQ(queue.top()->time, 300);
    queue.pop();
    EXPECT_TRUE(queue.empty());
}

TEST_F(EventQueueTest, PopOnEmptyQueueDoesNotCrash) {
    queue.pop();
    EXPECT_TRUE(queue.empty());
}

TEST_F(EventQueueTest, GetFirstEventTime) {
    queue.push(makeEvent(500));
    queue.push(makeEvent(200));
    queue.push(makeEvent(800));

    EXPECT_EQ(queue.getFirstEventTime(), 200);
}

TEST_F(EventQueueTest, ExtendFromSortedEventQueue) {
    SortedEventQueue sorted;
    sorted.push(makeEvent(50));
    sorted.push(makeEvent(150));

    queue.push(makeEvent(100));
    queue.extend(std::move(sorted));

    EXPECT_EQ(queue.size(), 3u);
    EXPECT_EQ(queue.getFirstEventTime(), 50);
}

TEST_F(EventQueueTest, ExtendFromVector) {
    EventList events = {
        makeEvent(400),
        makeEvent(200),
    };

    queue.extend(events);

    EXPECT_EQ(queue.size(), 2u);
    EXPECT_EQ(queue.getFirstEventTime(), 200);
}

TEST_F(EventQueueTest, ClearRemovesAllEvents) {
    queue.push(makeEvent(100));
    queue.push(makeEvent(200));
    queue.clear();

    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), 0u);
}

TEST_F(EventQueueTest, ManyEventsOrderedCorrectly) {
    for (int i = 100; i >= 1; --i) {
        queue.push(makeEvent(i));
    }
    EXPECT_EQ(queue.size(), 100u);

    int prev = 0;
    while (!queue.empty()) {
        EXPECT_GE(queue.top()->time, prev);
        prev = queue.top()->time;
        queue.pop();
    }
}

// --- lazy invalidation (cancel + reinsert) ---

TEST_F(EventQueueTest, CancelledEventIsSkippedByNextEvent) {
    auto a = makeEvent(100, des::EventType::MISSION_DISPATCH);
    auto b = makeEvent(300, des::EventType::MISSION_DISPATCH);
    queue.push(a);
    queue.push(b);

    a->cancelled = true;
    auto next = queue.nextEvent(des::EventType::MISSION_DISPATCH);
    ASSERT_NE(next, nullptr);
    EXPECT_EQ(next->time, 300);
}

TEST_F(EventQueueTest, CancelAndReinsertYieldsNewTimeAtFront) {
    auto a = makeEvent(200);
    queue.push(a);

    a->cancelled = true;
    queue.push(a->withTime(50));

    EXPECT_EQ(queue.top()->time, 50);
    EXPECT_FALSE(queue.top()->cancelled);
}

TEST_F(EventQueueTest, SameTimeEventsPopInPushOrder) {
    auto first = makeEvent(100, des::EventType::SIMULATION_START);
    auto second = makeEvent(100, des::EventType::SIMULATION_END);
    queue.push(first);
    queue.push(second);

    EXPECT_EQ(queue.top(), first); queue.pop();
    EXPECT_EQ(queue.top(), second);
}

// --- nextEvent ---

TEST_F(EventQueueTest, NextEventOnEmptyQueueReturnsNullptr) {
    EXPECT_EQ(queue.nextEvent(des::EventType::MISSION_DISPATCH), nullptr);
}

TEST_F(EventQueueTest, NextEventReturnsNullptrWhenTypeAbsent) {
    queue.push(makeEvent(100, des::EventType::SIMULATION_START));
    queue.push(makeEvent(200, des::EventType::SIMULATION_END));

    EXPECT_EQ(queue.nextEvent(des::EventType::MISSION_DISPATCH), nullptr);
}

TEST_F(EventQueueTest, NextEventReturnsEarliestOfMatchingType) {
    queue.push(makeEvent(500, des::EventType::MISSION_DISPATCH));
    queue.push(makeEvent(100, des::EventType::SIMULATION_START));
    queue.push(makeEvent(300, des::EventType::MISSION_DISPATCH));
    queue.push(makeEvent(400, des::EventType::MISSION_DISPATCH));
    queue.push(makeEvent(200, des::EventType::SIMULATION_END));

    auto next = queue.nextEvent(des::EventType::MISSION_DISPATCH);
    ASSERT_NE(next, nullptr);
    EXPECT_EQ(next->time, 300);
    EXPECT_EQ(next->getType(), des::EventType::MISSION_DISPATCH);
}

TEST_F(EventQueueTest, NextEventIgnoresEarlierEventsOfOtherTypes) {
    // Earliest event in the queue is a non-matching type — must be skipped.
    queue.push(makeEvent(100, des::EventType::SIMULATION_START));
    queue.push(makeEvent(250, des::EventType::MISSION_DISPATCH));

    auto next = queue.nextEvent(des::EventType::MISSION_DISPATCH);
    ASSERT_NE(next, nullptr);
    EXPECT_EQ(next->time, 250);
}

TEST_F(EventQueueTest, NextEventWithSingleMatchingEvent) {
    auto only = makeEvent(750, des::EventType::BATTERY_FULL);
    queue.push(only);

    EXPECT_EQ(queue.nextEvent(des::EventType::BATTERY_FULL), only);
}

TEST_F(EventQueueTest, NextEventFindsEarliestEvenWhenDeepInHeap) {
    // Crafted push order produces a heap where the earliest A is NOT at
    // the first A position when scanning the array linearly. A naive
    // first-match scan would return 200/A instead of the correct 100/A.
    queue.push(makeEvent(100, des::EventType::MISSION_DISPATCH));
    queue.push(makeEvent(50,  des::EventType::SIMULATION_END));
    queue.push(makeEvent(200, des::EventType::MISSION_DISPATCH));
    queue.push(makeEvent(10,  des::EventType::SIMULATION_END));

    auto next = queue.nextEvent(des::EventType::MISSION_DISPATCH);
    ASSERT_NE(next, nullptr);
    EXPECT_EQ(next->time, 100);
}

TEST_F(EventQueueTest, NextEventReflectsPopAndCancel) {
    auto a = makeEvent(100, des::EventType::MISSION_DISPATCH);
    auto b = makeEvent(300, des::EventType::MISSION_DISPATCH);
    queue.push(a);
    queue.push(b);

    EXPECT_EQ(queue.nextEvent(des::EventType::MISSION_DISPATCH)->time, 100);

    queue.pop(); // removes a
    EXPECT_EQ(queue.nextEvent(des::EventType::MISSION_DISPATCH)->time, 300);

    b->cancelled = true;
    EXPECT_EQ(queue.nextEvent(des::EventType::MISSION_DISPATCH), nullptr);
}
