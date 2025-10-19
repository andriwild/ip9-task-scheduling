#pragma once

#include <memory>
#include <queue>

class EventQueue;

class SimulationEvent {
protected:
    int time;
public:
    explicit SimulationEvent(const int t): time(t){}
    virtual ~SimulationEvent() = default;
    virtual void execute(EventQueue &eventQueue) = 0;
    int getTime() const { return time; };
};

struct EventComparator {
    bool operator()(const std::unique_ptr<SimulationEvent>& a,
                    const std::unique_ptr<SimulationEvent>& b) const {
        return a->getTime()> b->getTime();
    }
};

class EventQueue {
public:

    template<typename EventType, typename... Args>
    void addEvent(Args&&... args) {
        m_queue.push(std::make_unique<EventType>(std::forward<Args>(args)...));
    }

    void addEvent(std::unique_ptr<SimulationEvent> event) {
        m_queue.push(std::move(event));
    }

    bool hasEvents() const {
        return !m_queue.empty();
    }

    std::unique_ptr<SimulationEvent> getNextEvent() {
        auto event = std::move(const_cast<std::unique_ptr<SimulationEvent> &>(m_queue.top()));
        m_queue.pop();
        return event;
    }

    int getNextEventTime() const {
        return m_queue.top()->getTime();
    }

private:
    std::priority_queue<
        std::unique_ptr<SimulationEvent>,
        std::vector<std::unique_ptr<SimulationEvent>>,
        EventComparator> m_queue;
};