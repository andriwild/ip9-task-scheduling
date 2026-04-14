#pragma once

#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "../../util/types.h"

class ISimContext;
constexpr int ONE_HOUR = 3600;

class IEvent {
public:
    int time;
    explicit IEvent(const int time) : time(time) {}
    virtual ~IEvent() = default;

    virtual void execute(ISimContext& ctx) = 0;
    virtual std::string getName() const = 0;
    virtual des::EventType getType() const = 0;
    virtual std::string getColor() const { return ""; }

    bool operator<(const IEvent& other) const {
        return time < other.time;
    }

    friend std::ostream& operator<<(std::ostream& os, const IEvent& event) {
        os << "[" << event.time << "] " << event.getName();
        return os;
    }
};

struct EventComparator {
    bool operator()(const std::shared_ptr<IEvent>& a, const std::shared_ptr<IEvent>& b) const {
        if (!a || !b) {
            return false;
        }
        return a->time > b->time;
    }
};

using EventList = std::vector<std::shared_ptr<IEvent>>;

using SortedEventQueue = std::priority_queue<
    std::shared_ptr<IEvent>,
    EventList,
    EventComparator>;
