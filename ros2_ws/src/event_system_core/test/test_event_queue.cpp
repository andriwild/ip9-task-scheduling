#include <gtest/gtest.h>
#include <memory>

#include "engine/event.h"
#include "engine/event_queue.h"

// Minimal concrete event for testing (no ROS dependency)
class DummyEvent final : public des::IEvent {
    des::EventType m_type;
public:
    explicit DummyEvent(int time, des::EventType type = des::EventType::SIMULATION_START)
        : des::IEvent(time), m_type(type) {}
    void execute(des::ISimContext&) override {}
    std::string getName() const override { return "Dummy"; }
    des::EventType getType() const override { return m_type; }
    std::shared_ptr<des::IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<DummyEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }
};

class EventQueueTest : public ::testing::Test {
protected:
    des::EventQueue queue;

    static std::shared_ptr<des::IEvent> makeEvent(int time) {
        return std::make_shared<DummyEvent>(time);
    }

    static std::shared_ptr<des::IEvent> makeEvent(int time, des::EventType type) {
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

TEST_F(EventQueueTest, ExtendOntoNonEmptyQueue) {
    des::EventList events = {
        makeEvent(50),
        makeEvent(150),
    };

    queue.push(makeEvent(100));
    queue.extend(events);

    EXPECT_EQ(queue.size(), 3u);
    EXPECT_EQ(queue.getFirstEventTime(), 50);
}

TEST_F(EventQueueTest, ExtendFromVector) {
    des::EventList events = {
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

// --- deterministic tie-breaking via sequence number ---

TEST_F(EventQueueTest, ConstructionAssignsMonotonicSeq) {
    auto a = makeEvent(100);
    auto b = makeEvent(100);
    auto c = makeEvent(100);
    EXPECT_LT(a->seq, b->seq);
    EXPECT_LT(b->seq, c->seq);
}

TEST_F(EventQueueTest, SameTimeEventsPopInSeqOrderRegardlessOfPushOrder) {
    // Created first, but pushed last: order must follow seq (creation), not push.
    auto early = makeEvent(100, des::EventType::SIMULATION_START);
    auto late = makeEvent(100, des::EventType::SIMULATION_END);

    queue.push(late);
    queue.push(early);

    EXPECT_EQ(queue.top(), early); queue.pop();
    EXPECT_EQ(queue.top(), late);
}

TEST_F(EventQueueTest, TimeTakesPrecedenceOverSeq) {
    auto laterSeqEarlierTime = makeEvent(100);
    auto earlierSeqLaterTime = makeEvent(300);
    // earlierSeqLaterTime has the smaller seq but the larger time.
    queue.push(earlierSeqLaterTime);
    queue.push(laterSeqEarlierTime);

    EXPECT_EQ(queue.top()->time, 100); queue.pop();
    EXPECT_EQ(queue.top()->time, 300);
}

TEST_F(EventQueueTest, ExtendPreservesSeqOrderForSameTime) {
    auto e0 = makeEvent(100, des::EventType::MISSION_DISPATCH);
    auto e1 = makeEvent(100, des::EventType::SIMULATION_END);
    auto e2 = makeEvent(100, des::EventType::SIMULATION_START);

    des::EventList scrambled = {e2, e0, e1};

    queue.extend(scrambled);

    EXPECT_EQ(queue.top(), e0); queue.pop();
    EXPECT_EQ(queue.top(), e1); queue.pop();
    EXPECT_EQ(queue.top(), e2);
}

TEST_F(EventQueueTest, WithTimeAssignsFreshSeq) {
    auto original = makeEvent(100);
    auto rescheduled = original->withTime(100); // same time, must sort after
    EXPECT_GT(rescheduled->seq, original->seq);

    queue.push(rescheduled);
    queue.push(original);
    EXPECT_EQ(queue.top(), original);
}
