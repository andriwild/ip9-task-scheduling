#include "engine/event_queue.h"

#include "util/log.h"

namespace des {

void EventQueue::extend(std::vector<std::shared_ptr<IEvent>> events) {
    for (const auto& event : events) {
        m_events.push(event);
    }
}

bool EventQueue::empty() const {
    return m_events.empty();
}

size_t EventQueue::size() const {
    return m_events.size();
}

void EventQueue::push(const std::shared_ptr<IEvent>& event) {
    m_events.push(event);
}

void EventQueue::pop() {
    if (m_events.empty()) {
        return;
    }
    m_events.pop();
}

std::shared_ptr<IEvent> EventQueue::top() const {
    return m_events.empty() ? nullptr : m_events.top();
}

void EventQueue::clear() {
    m_events = {};
}

int EventQueue::getFirstEventTime() const {
    const auto t = top();
    return t ? t->time : 0;
}

}  // namespace des
