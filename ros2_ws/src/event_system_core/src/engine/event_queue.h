/*
 * Priority queue of pending simulation events.
 * Ordering key is (time, priority desc, insertion sequence),
 * which makes the event order deterministic for a given seed.
 *
 */

#pragma once

#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "engine/contracts/i_event.h"
#include "util/types.h"

namespace des {

class EventQueue {
    struct EarliestFirst {
        bool operator()(const std::shared_ptr<IEvent>& a,
                        const std::shared_ptr<IEvent>& b) const {
            if (a->time != b->time) {
                return a->time < b->time;
            }
            if (a->priority() != b->priority()) {
                return a->priority() > b->priority();
            }
            return a->seq < b->seq;
        }
    };

    std::multiset<std::shared_ptr<IEvent>, EarliestFirst> m_events;

public:
    explicit EventQueue() = default;

    void extend(std::vector<std::shared_ptr<IEvent>> events);

    bool empty() const;

    [[nodiscard]] size_t size() const;

    void push(const std::shared_ptr<IEvent>& event);

    void pop();

    std::shared_ptr<IEvent> top() const;

    void clear();

    int getFirstEventTime() const;

    std::shared_ptr<IEvent> nextEvent(EventType type) const;

    // Set a flag on the given event
    void cancelByType(EventType type) const;

    // Time of the earliest queued event matching `type`
    std::optional<int> nextEventTime(EventType type) const;

    // Convenience: next MissionDispatchEvent in the queue
    std::optional<int> nextDispatchTime() const;

    std::shared_ptr<IEvent> nextDispatchEvent() const;

    std::vector<std::shared_ptr<IEvent>> dispatchEventsUntil(int untilTime) const;

    void print() const;
};

}  // namespace des
