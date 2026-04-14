#include <gtest/gtest.h>
#include <memory>

#include "../src/model/event.h"
#include "../src/model/event_queue.h"

// Minimal concrete event for testing (no ROS dependency)
class DummyEvent final : public IEvent {
public:
    explicit DummyEvent(int time) : IEvent(time) {}
    void execute(ISimContext&) override {}
    std::string getName() const override { return "Dummy"; }
    des::EventType getType() const override { return des::EventType::SIMULATION_START; }
};

class EventQueueTest : public ::testing::Test {
protected:
    EventQueue queue;

    static std::shared_ptr<IEvent> makeEvent(int time) {
        return std::make_shared<DummyEvent>(time);
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

TEST_F(EventQueueTest, GetLastEventTimeTracksMaximum) {
    queue.push(makeEvent(100));
    queue.push(makeEvent(500));
    queue.push(makeEvent(300));

    EXPECT_EQ(queue.getLastEventTime(), 500);
}

TEST_F(EventQueueTest, GetLastEventTimeResetsAfterFullDrain) {
    queue.push(makeEvent(100));
    queue.pop();
    EXPECT_EQ(queue.getLastEventTime(), 0);
}

TEST_F(EventQueueTest, ExtendFromSortedEventQueue) {
    SortedEventQueue sorted;
    sorted.push(makeEvent(50));
    sorted.push(makeEvent(150));

    queue.push(makeEvent(100));
    queue.extend(std::move(sorted));

    EXPECT_EQ(queue.size(), 3u);
    EXPECT_EQ(queue.getFirstEventTime(), 50);
    EXPECT_EQ(queue.getLastEventTime(), 150);
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

TEST_F(EventQueueTest, LastEventTimeUpdatedByExtendSortedQueue) {
    queue.push(makeEvent(100));

    SortedEventQueue sorted;
    sorted.push(makeEvent(999));
    queue.extend(std::move(sorted));

    EXPECT_EQ(queue.getLastEventTime(), 999);
}

TEST_F(EventQueueTest, ExtendFromVectorUpdatesLastEventTime) {
    EventList events = {
        makeEvent(400),
        makeEvent(800),
    };

    queue.extend(events);

    EXPECT_EQ(queue.getLastEventTime(), 800);
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
