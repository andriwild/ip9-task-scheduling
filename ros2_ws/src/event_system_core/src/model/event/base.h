#pragma once

#include <atomic>
#include <cstdint>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "../../util/types.h"

class ISimContext;
constexpr int ONE_HOUR = 3600;

class IEvent {
    static inline std::atomic<uint64_t> s_seqCounter{0};

public:
    int time;
    uint64_t seq;
    bool cancelled = false;
    explicit IEvent(const int time) : time(time), seq(s_seqCounter.fetch_add(1, std::memory_order_relaxed)) {}
    IEvent(const IEvent& o) : time(o.time), seq(s_seqCounter.fetch_add(1, std::memory_order_relaxed)), cancelled(o.cancelled) {}
    virtual ~IEvent() = default;

    virtual void execute(ISimContext& ctx) = 0;
    virtual std::string getName() const = 0;
    virtual des::EventType getType() const = 0;
    virtual std::shared_ptr<IEvent> withTime(int newTime) const = 0;
    virtual std::string getColor() const { return ""; }
    virtual int getMissionId() const { return -1; }

    bool operator<(const IEvent& other) const {
        if (time != other.time) {
            return time < other.time;
        }
        return seq < other.seq;
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
        if (a->time != b->time) {
            return a->time > b->time;
        }
        return a->seq > b->seq;
    }
};

using EventList = std::vector<std::shared_ptr<IEvent>>;

using SortedEventQueue = std::priority_queue<
    std::shared_ptr<IEvent>,
    EventList,
    EventComparator>;
