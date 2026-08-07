#include "engine/event_queue.h"

#include <rclcpp/rclcpp.hpp>

#include "util/log.h"

namespace des {

void EventQueue::extend(SortedEventQueue queue) {
    while (!queue.empty()) {
        m_events.insert(queue.top());
        queue.pop();
    }
}

void EventQueue::extend(std::vector<std::shared_ptr<IEvent>> events) {
    for (const auto& event : events) {
        m_events.insert(event);
    }
}

bool EventQueue::empty() const {
    return m_events.empty();
}

size_t EventQueue::size() const {
    return m_events.size();
}

void EventQueue::push(const std::shared_ptr<IEvent>& event) {
    m_events.insert(event);
}

void EventQueue::pop() {
    if (m_events.empty()) {
        return;
    }
    m_events.erase(m_events.begin());
}

std::shared_ptr<IEvent> EventQueue::top() const {
    return m_events.empty() ? nullptr : *m_events.begin();
}

void EventQueue::clear() {
    m_events.clear();
}

int EventQueue::getFirstEventTime() const {
    const auto t = top();
    return t ? t->time : 0;
}

std::shared_ptr<IEvent> EventQueue::nextEvent(const EventType type) const {
    for (const auto& e : m_events) {
        if (e->getType() == type && !e->cancelled) {
            return e;
        }
    }
    return nullptr;
}

void EventQueue::cancelByType(const EventType type) const {
    for (const auto& e : m_events) {
        if (e->getType() == type && !e->cancelled) {
            e->cancelled = true;
        }
    }
}

std::optional<int> EventQueue::nextEventTime(const EventType type) const {
    const auto e = nextEvent(type);
    return e ? std::optional<int>(e->time) : std::nullopt;
}

std::optional<int> EventQueue::nextDispatchTime() const {
    return nextEventTime(EventType::MISSION_DISPATCH);
}

std::shared_ptr<IEvent> EventQueue::nextDispatchEvent() const {
    return nextEvent(EventType::MISSION_DISPATCH);
}

std::vector<std::shared_ptr<IEvent>> EventQueue::dispatchEventsUntil(const int untilTime) const {
    std::vector<std::shared_ptr<IEvent>> events;
    for (const auto& e : m_events) {
        if (e->time > untilTime) {
            break;
        }
        if (e->getType() == EventType::MISSION_DISPATCH && !e->cancelled) {
            events.push_back(e);
        }
    }
    return events;
}

void EventQueue::print() const {
    for (const auto& e : m_events) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.event_queue"), "%d: %d - %s%s", e->time, static_cast<int>(e->getType()), e->getName().c_str(), e->cancelled ? " (cancelled)" : "");
    }
}

}  // namespace des
