#pragma once

#include <memory>

class EventQueue;

class SimulationEvent {
protected:
    int time;
public:
    explicit SimulationEvent(const int t): time(t){}
    virtual ~SimulationEvent() = default;
    virtual void execute(EventQueue &eventQueue, bool randomness = true) = 0;
    int getTime() const { return time; };
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