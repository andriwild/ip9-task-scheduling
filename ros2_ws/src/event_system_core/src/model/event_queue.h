#pragma once

#include <algorithm>
#include <cassert>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "../model/event/base.h"
#include "../util/types.h"

class EventQueue {
    // Vector-backed min-heap (earliest time on top). We don't use
    // std::priority_queue here because reschedule() needs to find and
    // re-prioritise an arbitrary entry — only possible with direct vector access.
    EventList m_heap;
    EventComparator m_cmp;
    int m_lastEventTime = 0;

    void heapPush(const std::shared_ptr<IEvent>& event) {
        m_heap.push_back(event);
        std::push_heap(m_heap.begin(), m_heap.end(), m_cmp);
        if (event->time > m_lastEventTime) {
            m_lastEventTime = event->time;
        }
    }

public:
    explicit EventQueue() = default;

    void extend(SortedEventQueue queue) {
        while(!queue.empty()){
            heapPush(queue.top());
            queue.pop();
        }
    }

    void extend(std::vector<std::shared_ptr<IEvent>> events) {
        for (const auto& event: events) {
            heapPush(event);
        }
    }

    bool empty() const { return m_heap.empty(); }

    [[nodiscard]] size_t size() const { return m_heap.size(); }

    void push(const std::shared_ptr<IEvent>& event) {
        heapPush(event);
    }

    void pop() {
        if (m_heap.empty()) { return; }
        std::pop_heap(m_heap.begin(), m_heap.end(), m_cmp);
        m_heap.pop_back();
        if (m_heap.empty()) {
            m_lastEventTime = 0;
        }
    }

    std::shared_ptr<IEvent> top() const {
        return m_heap.empty() ? nullptr : m_heap.front();
    }

    void clear() {
        m_heap.clear();
        m_lastEventTime = 0;
    }

    // Shifts `event`'s time to `newTime` and re-heaps. Returns false if the
    // event isn't in the queue (already popped, never pushed). Used by the
    // interrupt-shift mechanism — the robot's in-flight commitment gets
    // pushed back by the interrupt duration.
    bool reschedule(const std::shared_ptr<IEvent>& event, int newTime) {
        auto it = std::find(m_heap.begin(), m_heap.end(), event);
        if (it == m_heap.end()) { return false; }
        const int oldTime = (*it)->time;
        (*it)->time = newTime;
        std::make_heap(m_heap.begin(), m_heap.end(), m_cmp);
        if (newTime > m_lastEventTime) {
            m_lastEventTime = newTime;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("EventQueue"),
                     "Reschedule '%s': %d → %d (Δ%+d)",
                     event->getName().c_str(), oldTime, newTime, newTime - oldTime);
        return true;
    }

    int getLastEventTime() const {
        return m_lastEventTime;
    }

    int getFirstEventTime() const {
        auto t = top();
        return t ? t->time : 0;
    }

    // Earliest queued event matching `type`, or nullptr if none. O(n).
    std::shared_ptr<IEvent> nextEvent(des::EventType type) const {
        std::shared_ptr<IEvent> best;
        for (const auto& e : m_heap) {
            if (e->getType() != type) { continue; }
            if (!best || e->time < best->time) { best = e; }
        }
        return best;
    }

    // Time of the earliest queued event matching `type`. nullopt = none.
    std::optional<int> nextEventTime(des::EventType type) const {
        auto e = nextEvent(type);
        return e ? std::optional<int>(e->time) : std::nullopt;
    }

    // Convenience: next MissionDispatchEvent in the queue.
    std::optional<int> nextDispatchTime() const {
        return nextEventTime(des::EventType::MISSION_DISPATCH);
    }
    std::shared_ptr<IEvent> nextDispatchEvent() const {
        return nextEvent(des::EventType::MISSION_DISPATCH);
    }

    void print() const {
        auto copy = m_heap;
        std::sort(copy.begin(), copy.end(),
                  [](const auto& a, const auto& b) { return a->time < b->time; });
        for (const auto& e : copy) {
            std::cout << e->time << ": " << static_cast<int>(e->getType()) << " - " << e->getName() << std::endl;
        }
    }
};
