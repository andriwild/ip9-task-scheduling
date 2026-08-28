// Generated with Claude Code (Anthropic), then reviewed and adapted by the author. See the index of auxiliary tools.
#pragma once

#include <chrono>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>

#include "util/constants.h"
#include "util/log.h"

namespace des {

class PerfProfiler {
    using Clock = std::chrono::steady_clock;

    struct DayRow {
        int day = 0;
        long long popped = 0;
        long long executed = 0;
        double wallSeconds = 0.0;
        std::size_t queueSize = 0;
        long long protocolSize = 0;
        double rssMegabytes = 0.0;
        int rounds = 0;
    };

    bool m_enabled = false;
    bool m_started = false;
    std::string m_path;
    int m_day = 0;
    long long m_popped = 0;
    long long m_executed = 0;
    Clock::time_point m_dayStart;
    std::map<int, DayRow> m_rows;

    static double currentRssMegabytes() {
        std::ifstream in("/proc/self/statm");
        long long pages = 0;
        long long resident = 0;
        if (!(in >> pages >> resident)) {
            return 0.0;
        }
        return static_cast<double>(resident) * 4096.0 / (1024.0 * 1024.0);
    }

    void closeDay(const std::size_t queueSize, const long long protocolSize) {
        const double seconds = std::chrono::duration<double>(Clock::now() - m_dayStart).count();
        DayRow& row = m_rows[m_day];
        row.day = m_day;
        row.popped += m_popped;
        row.executed += m_executed;
        row.wallSeconds += seconds;
        row.queueSize = queueSize;
        row.protocolSize = protocolSize;
        row.rssMegabytes = currentRssMegabytes();
        ++row.rounds;
        m_popped = 0;
        m_executed = 0;
        m_dayStart = Clock::now();
    }

public:
    void enable(std::string path) {
        m_enabled = true;
        m_path = std::move(path);
    }

    [[nodiscard]] bool enabled() const {
        return m_enabled;
    }

    void onEvent(const int simTime, const bool executed, const std::size_t queueSize, const long long protocolSize) {
        if (!m_enabled) {
            return;
        }
        const int day = simTime / SECONDS_PER_DAY;
        if (!m_started) {
            m_started = true;
            m_day = day;
            m_dayStart = Clock::now();
        }
        if (day != m_day) {
            closeDay(queueSize, protocolSize);
            m_day = day;
        }
        ++m_popped;
        if (executed) {
            ++m_executed;
        }
    }

    void finish(const std::size_t queueSize, const long long protocolSize) {
        if (!m_enabled || !m_started) {
            return;
        }
        closeDay(queueSize, protocolSize);
        m_started = false;

        std::ofstream out(m_path);
        if (!out) {
            DES_LOG_ERROR("des.perf", "Could not write %s", m_path.c_str());
            return;
        }
        out << "day,rounds,popped_events,executed_events,wall_seconds,popped_per_second,executed_per_second,queue_size,protocol_size,rss_mb\n";
        for (const auto& entry : m_rows) {
            const DayRow& r = entry.second;
            const double poppedRate = r.wallSeconds > 0.0 ? static_cast<double>(r.popped) / r.wallSeconds : 0.0;
            const double executedRate = r.wallSeconds > 0.0 ? static_cast<double>(r.executed) / r.wallSeconds : 0.0;
            out << r.day << ','
                << r.rounds << ','
                << r.popped << ','
                << r.executed << ','
                << r.wallSeconds << ','
                << poppedRate << ','
                << executedRate << ','
                << r.queueSize << ','
                << r.protocolSize << ','
                << r.rssMegabytes << '\n';
        }
        DES_LOG_INFO("des.perf", "Wrote %zu day rows to %s", m_rows.size(), m_path.c_str());
        m_rows.clear();
    }
};

}  // namespace des
