#pragma once

#include <atomic>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "model/order.h"
#include "util/types.h"

namespace des {

class ISimContext;

class IEvent {
    static inline std::atomic<uint64_t> s_seqCounter{0};

public:
    int time;
    uint64_t seq;
    bool cancelled = false;
    explicit IEvent(const int time) : time(time), seq(s_seqCounter.fetch_add(1, std::memory_order_relaxed)) {}
    IEvent(const IEvent& o) : time(o.time), seq(s_seqCounter.fetch_add(1, std::memory_order_relaxed)), cancelled(o.cancelled), m_missionId(o.m_missionId) {}
    virtual ~IEvent() = default;

    virtual void execute(ISimContext& ctx) = 0;
    virtual bool isStale(const ISimContext&) const { return false; }
    virtual std::string getName() const = 0;
    virtual EventType getType() const = 0;
    virtual std::shared_ptr<IEvent> withTime(int newTime) const = 0;
    virtual std::string getColor() const { return ""; }
    virtual int getMissionId() const { return m_missionId; }
    void setMissionId(const int missionId) { m_missionId = missionId; }
    virtual OrderPtr getOrder() const { return nullptr; }
    virtual double getDistance() const { return 0.0; }
    virtual int priority() const { return 0; }

private:
    int m_missionId = -1;

public:
    bool operator<(const IEvent& other) const {
        if (time != other.time) {
            return time < other.time;
        }
        if (priority() != other.priority()) {
            return priority() > other.priority();
        }
        return seq < other.seq;
    }

    friend std::ostream& operator<<(std::ostream& os, const IEvent& event) {
        os << "[" << event.time << "] " << event.getName();
        return os;
    }
};

using EventList = std::vector<std::shared_ptr<IEvent>>;

}  // namespace des
