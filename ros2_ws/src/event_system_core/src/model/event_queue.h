#pragma once

#include <optional>
#include <set>
#include <rclcpp/rclcpp.hpp>

#include "../util/log.h"
#include "../model/event/base.h"
#include "../util/types.h"

class EventQueue {
    struct EarliestFirst {
        bool operator()(const std::shared_ptr<IEvent>& a,
                        const std::shared_ptr<IEvent>& b) const {
            return a->time < b->time;
        }
    };

    std::multiset<std::shared_ptr<IEvent>, EarliestFirst> m_events;

public:
    explicit EventQueue() = default;

    void extend(SortedEventQueue queue) {
        while(!queue.empty()){
            m_events.insert(queue.top());
            queue.pop();
        }
    }

    void extend(std::vector<std::shared_ptr<IEvent>> events) {
        for (const auto& event: events) {
            m_events.insert(event);
        }
    }

    bool empty() const { return m_events.empty(); }

    [[nodiscard]] size_t size() const { return m_events.size(); }

    void push(const std::shared_ptr<IEvent>& event) {
        m_events.insert(event);
    }

    void pop() {
        if (m_events.empty()) { return; }
        m_events.erase(m_events.begin());
    }

    std::shared_ptr<IEvent> top() const {
        return m_events.empty() ? nullptr : *m_events.begin();
    }

    void clear() {
        m_events.clear();
    }

    int getFirstEventTime() const {
        auto t = top();
        return t ? t->time : 0;
    }

    std::shared_ptr<IEvent> nextEvent(des::EventType type) const {
        for (const auto& e : m_events) {
            if (e->getType() == type && !e->cancelled) {
                return e;
            }
        }
        return nullptr;
    }

    void cancelByType(des::EventType type) const {
        for (const auto& e : m_events) {
            if (e->getType() == type && !e->cancelled) {
                e->cancelled = true;
            }
        }
    }

    // Time of the earliest queued event matching `type`
    std::optional<int> nextEventTime(des::EventType type) const {
        auto e = nextEvent(type);
        return e ? std::optional<int>(e->time) : std::nullopt;
    }

    // Convenience: next MissionDispatchEvent in the queue
    std::optional<int> nextDispatchTime() const {
        return nextEventTime(des::EventType::MISSION_DISPATCH);
    }

    std::shared_ptr<IEvent> nextDispatchEvent() const {
        return nextEvent(des::EventType::MISSION_DISPATCH);
    }

    void print() const {
        for (const auto& e : m_events) {
            DES_LOG_INFO(rclcpp::get_logger("des.event_queue"), "%d: %d - %s%s", e->time, static_cast<int>(e->getType()), e->getName().c_str(), e->cancelled ? " (cancelled)" : "");
        }
    }
};
