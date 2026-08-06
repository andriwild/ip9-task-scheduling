/**
*
* Keep track of the robots state.
* Stores to each state the duration for metrics.
*
**/

#pragma once

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "../util/types.h"

struct StateInterval {
    int time;
    int endTime;
    des::RobotStateType category;
    std::string name;
    double socFrom;
    double socTo;
};

class StateLog {
    std::vector<StateInterval> m_entries;
    bool m_open = false;

public:
    void open(const int time, const des::RobotStateType category, std::string name, const double soc) {
        if (m_open) {
            close(time, soc);
        }
        m_entries.push_back(StateInterval{ time, time, category, std::move(name), soc, soc });
        m_open = true;
    }

    void close(const int time, const double soc) {
        if (!m_open) {
            return;
        }
        m_open = false;
        StateInterval& last = m_entries.back();
        last.endTime = std::max(time, last.time);
        last.socTo   = soc;
    }

    bool isOpen() const {
        return m_open;
    }

    const std::vector<StateInterval>& entries() const {
        return m_entries;
    }

    std::size_t size() const {
        return m_entries.size();
    }

    void clear() {
        m_entries.clear();
        m_open = false;
    }
};
