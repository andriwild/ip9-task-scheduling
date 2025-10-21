#pragma once

#include <memory>

#include "tree.h"

class EventQueue;


inline static int nextId;
class SimulationEvent {
protected:
    int m_id;
    int m_time;
public:
    explicit SimulationEvent(const int t):
    m_id(nextId++),
    m_time(t)
    {}
    virtual ~SimulationEvent() = default;
    virtual void execute(Robot &robot, Tree<SimulationEvent> &eventQueue, bool randomness = true) = 0;
    int getTime() const { return m_time; };
    virtual std::string getName() const  = 0;
    int getId() const {
        return m_id;
    }

    friend std::ostream& operator<<(std::ostream& os, const SimulationEvent& event) {
        os << "[" << event.getId() << "] " << event.getName() << " (t=" << event.m_time << ")";
        return os;
    }
};


class EventQueue {
public:
    EventQueue() = default;

    void addEvent(std::unique_ptr<SimulationEvent> event) {
        m_events.push_back(std::move(event));
        sortEvents();
    }

    template<typename EventType, typename... Args>
    void addEvent(Args&&... args) {
        m_events.push_back(std::make_unique<EventType>(std::forward<Args>(args)...));
        sortEvents();
    }

    bool hasEvents() const {
        return !m_events.empty();
    }

    std::unique_ptr<SimulationEvent> getNextEvent() {
        if (m_events.empty()) return nullptr;
        auto event = std::move(m_events.front());
        m_events.erase(m_events.begin());
        return event;
    }

    int getNextEventTime() const {
        return m_events.empty() ? -1 : m_events.front()->getTime();
    }

    const std::vector<std::unique_ptr<SimulationEvent>>& getEvents() const {
        return m_events;
    }

    std::vector<SimulationEvent*> getAllEvents() const {
        std::vector<SimulationEvent*> result;
        result.reserve(m_events.size());
        for (const auto& event : m_events) {
            result.push_back(event.get());
        }
        return result;
    }

private:
    void sortEvents() {
        std::ranges::sort(
            m_events,
            [](const auto& a, const auto& b) { return a->getTime() < b->getTime();}
            );
    }
    std::vector<std::unique_ptr<SimulationEvent>> m_events;
};