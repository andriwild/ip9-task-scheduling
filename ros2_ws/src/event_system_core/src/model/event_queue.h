#pragma once

#include <cassert>

#include "event.h"


class EventQueue {
    SortedEventQueue m_queue;
    int m_lastEventTime = 0;


public:
    explicit EventQueue() = default;

    void extend(SortedEventQueue queue) {
        while(!queue.empty()){
            auto item = queue.top();
            if (item->time > m_lastEventTime) {
                m_lastEventTime = item->time;
            }
            m_queue.push(std::move(item));
            queue.pop();
        }
    }

    void extend(std::vector<std::shared_ptr<IEvent>> events) {
        for (const auto& event: events) {
            if (event->time > m_lastEventTime) {
                m_lastEventTime = event->time;
            }
            m_queue.push(std::move(event));
        }
    }

    bool empty() const { return m_queue.empty(); }

    [[nodiscard]] size_t size() const { return m_queue.size(); }

    void push(const std::shared_ptr<IEvent>& event) {
        if (event->time > m_lastEventTime) {
            m_lastEventTime = event->time;
        }
        m_queue.push(event);
    }

    void pop() {
        if(!m_queue.empty()) {
            m_queue.pop();

            if (m_queue.empty()){
                m_lastEventTime = 0;
            }
        }
    }

    std::shared_ptr<IEvent> top() const {
        return m_queue.empty() ? nullptr : m_queue.top();
    }

    void clear() {
        while(!m_queue.empty()) {
            m_queue.pop();
        }
    }

    int getLastEventTime() const {
        return m_lastEventTime;
    }

    int getFirstEventTime() const {
        return top()->time;
    }

    void print() const {
        auto queue = m_queue;
        while (!queue.empty()) {
            std::cout << queue.top()->time  << ": " << static_cast<int>(queue.top()->getType()) << " - " << queue.top()->getName() << std::endl;
            queue.pop();
        }
    }

};
