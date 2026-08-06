#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "engine/contracts/i_sim_context.h"
#include "engine/contracts/i_event.h"
#include "metrics/metrics_reduce.h"
#include "model/robot.h"
#include "util/log.h"
#include "util/types.h"

namespace des::metrics {

struct RunReport {
    int idleTime      = 0;
    int movingTime    = 0;
    int searchingTime = 0;
    int accompanyTime = 0;
    int chargingTime  = 0;
    int talkTime      = 0;

    int scheduledTotal     = 0;
    int scheduledOnTime    = 0;
    int scheduledLate      = 0;
    int scheduledFailed    = 0;
    int scheduledCancelled = 0;
    int scheduledRejected  = 0;

    int backgroundTotal     = 0;
    int backgroundCompleted = 0;
    int backgroundFailed    = 0;
    int backgroundCancelled = 0;

    int interruptTotal     = 0;
    int interruptCompleted = 0;

    float avgEarlyArrival = 0.0f;
    float avgLateness     = 0.0f;
    int   minLateness     = 0;
    int   maxLateness     = 0;
    float rejectedRate    = 0.0f;

    float totalDistance         = 0.0f;
    float energyTotalConsumedAh = 0.0f;
    float energyIdleAh          = 0.0f;
    float energySearchingAh     = 0.0f;
    float energyAccompanyAh     = 0.0f;
    float energyTalkAh          = 0.0f;
    float energyChargingAh      = 0.0f;

    float utilization     = 0.0f;
    float idlePercent     = 0.0f;
    float chargingPercent = 0.0f;

    int   chargeCyclesTotal    = 0;
    int   chargeCyclesComplete = 0;
    int   chargeCyclesPartial  = 0;
    int   deepDischargeCount   = 0;
    float equivalentFullCycles = 0.0f;
    float avgDepthOfDischarge  = 0.0f;
};

class MetricsReporter {
    using Row = std::vector<std::pair<std::string, std::string>>;

    struct Reduced {
        des::reduce::StateTotals   states;
        des::reduce::BatteryTotals battery;
        des::reduce::MissionTotals missions;
        des::reduce::DriveTotals   drives;
        std::map<int, des::reduce::DaySpan> days;
        std::map<int, int> chargeCyclesByDay;
    };

    std::shared_ptr<const des::SimConfig> m_config;
    std::string m_csvPath;
    std::string m_dailyCsvPath;
    std::string m_scenario;
    std::string m_runId;
    std::string m_terminatedReason = "completed";
    int m_round        = 0;
    int m_runIndex     = 0;
    unsigned int m_roundSeed = 0;

public:
    void enableCsv(std::string path, std::shared_ptr<const des::SimConfig> config) {
        m_csvPath = std::move(path);
        m_config  = std::move(config);
    }

    void enableDailyCsv(std::string path) {
        m_dailyCsvPath = std::move(path);
    }

    void setRunId(const std::string& runId) {
        m_runId = runId;
    }

    void setTerminatedReason(const std::string& reason) {
        m_terminatedReason = reason;
    }

    void setRunInfo(const std::string& scenario, const int round, const unsigned int roundSeed) {
        m_scenario  = scenario;
        m_round     = round;
        m_roundSeed = roundSeed;
    }

    void report(const ISimContext& ctx, const EventList& protocol) {
        if (!ctx.getRobot()) {
            return;
        }
        const Reduced r     = reduce(ctx, protocol);
        const RunReport rep = build(r);
        logSummary(rep, r);
        if (!m_csvPath.empty() && m_config) {
            appendCsv(m_csvPath, { runRow(rep) });
            appendCsv(m_dailyCsvPath, dailyRows(r));
        }
    }

private:
    static Reduced reduce(const ISimContext& ctx, const EventList& protocol) {
        const auto* robot = ctx.getRobot();
        const auto bat    = robot->batteryStats();
        Reduced r;
        r.states   = des::reduce::states(robot->getStateLog(), bat.capacity);
        r.battery  = des::reduce::battery(robot->getStateLog(), robot->getChargeSessions(), bat.capacity, bat.lowThreshold, robot->getDischargedAh());
        r.missions = des::reduce::missions(protocol);
        r.drives   = des::reduce::drives(protocol);
        r.days     = des::reduce::perDay(robot->getStateLog(), bat.capacity);
        r.chargeCyclesByDay = des::reduce::chargeCyclesByDay(robot->getChargeSessions());
        return r;
    }

    static des::reduce::MissionStats statsFor(const des::reduce::MissionTotals& m, const des::ExecutionMode mode) {
        const auto it = m.byMode.find(mode);
        return it == m.byMode.end() ? des::reduce::MissionStats{} : it->second;
    }

    static des::reduce::MissionStats scheduledOf(const std::map<des::ExecutionMode, des::reduce::MissionStats>& byMode) {
        const auto it = byMode.find(des::ExecutionMode::SCHEDULED);
        return it == byMode.end() ? des::reduce::MissionStats{} : it->second;
    }

    static int failCount(const des::reduce::MissionStats& s, const std::string& detail) {
        const auto it = s.failByDetail.find(detail);
        return it == s.failByDetail.end() ? 0 : it->second;
    }

    static RunReport build(const Reduced& r) {
        RunReport rep;

        auto timeFor = [&](const std::string& n) {
            const auto it = r.states.timeByName.find(n);
            return it == r.states.timeByName.end() ? 0 : it->second;
        };
        auto energyFor = [&](const std::string& n) {
            const auto it = r.states.energyByName.find(n);
            return it == r.states.energyByName.end() ? 0.0 : it->second;
        };
        auto categoryTime = [&](const des::RobotStateType c) {
            const auto it = r.states.timeByCategory.find(c);
            return it == r.states.timeByCategory.end() ? 0 : it->second;
        };

        rep.idleTime      = timeFor("idle");
        rep.movingTime    = r.drives.movingTime;
        rep.searchingTime = timeFor("search");
        rep.accompanyTime = timeFor("accompany");
        rep.chargingTime  = timeFor("charging");
        rep.talkTime      = timeFor("conversate");

        const auto sched = statsFor(r.missions, des::ExecutionMode::SCHEDULED);
        const auto bg    = statsFor(r.missions, des::ExecutionMode::BACKGROUND);
        const auto intr  = statsFor(r.missions, des::ExecutionMode::INTERRUPT);

        rep.scheduledTotal     = sched.registered;
        rep.scheduledOnTime    = sched.onTime;
        rep.scheduledLate      = sched.late;
        rep.scheduledFailed    = sched.failed;
        rep.scheduledCancelled = sched.cancelled;
        rep.scheduledRejected  = sched.rejected;

        rep.backgroundTotal     = bg.registered;
        rep.backgroundCompleted = bg.onTime + bg.late;
        rep.backgroundFailed    = bg.failed;
        rep.backgroundCancelled = bg.cancelled;

        rep.interruptTotal     = intr.registered;
        rep.interruptCompleted = intr.onTime + intr.late;

        rep.avgEarlyArrival = (sched.onTime > 0) ? static_cast<float>(r.missions.accEarlyTime) / sched.onTime : 0.0f;
        rep.avgLateness     = (sched.late > 0)   ? static_cast<float>(r.missions.accLateTime)  / sched.late   : 0.0f;
        rep.minLateness     = r.missions.hasLate ? r.missions.minLateness : 0;
        rep.maxLateness     = r.missions.hasLate ? r.missions.maxLateness : 0;
        rep.rejectedRate    = (sched.registered > 0) ? static_cast<float>(sched.rejected) / sched.registered : 0.0f;

        rep.energyIdleAh          = static_cast<float>(energyFor("idle"));
        rep.energySearchingAh     = static_cast<float>(energyFor("search"));
        rep.energyAccompanyAh     = static_cast<float>(energyFor("accompany"));
        rep.energyTalkAh          = static_cast<float>(energyFor("conversate"));
        rep.energyChargingAh      = static_cast<float>(energyFor("charging"));
        rep.energyTotalConsumedAh = static_cast<float>(r.battery.dischargedAh);

        rep.totalDistance = static_cast<float>(r.drives.distance);

        if (r.states.totalTime > 0) {
            const double total = r.states.totalTime;
            rep.utilization     = static_cast<float>(categoryTime(des::RobotStateType::MISSION)  / total * 100.0);
            rep.idlePercent     = static_cast<float>(timeFor("idle")                             / total * 100.0);
            rep.chargingPercent = static_cast<float>(categoryTime(des::RobotStateType::CHARGING) / total * 100.0);
        }

        rep.chargeCyclesTotal    = r.battery.cyclesTotal;
        rep.chargeCyclesComplete = r.missions.chargeCyclesComplete;
        rep.chargeCyclesPartial  = std::max(0, r.battery.cyclesTotal - r.missions.chargeCyclesComplete);
        rep.deepDischargeCount   = r.battery.deepDischarge;
        rep.equivalentFullCycles = static_cast<float>(r.battery.equivalentFullCycles);
        rep.avgDepthOfDischarge  = static_cast<float>(r.battery.avgDepthOfDischarge);

        return rep;
    }

    static void logSummary(const RunReport& m, const Reduced& r) {
        const auto log = rclcpp::get_logger("des.metrics.summary");
        DES_LOG_INFO(log, "================= SIMULATION SUMMARY =================");
        DES_LOG_INFO(log, "Scheduled  : total=%d  on-time=%d  late=%d  failed=%d  rejected=%d  (reject-rate=%.1f%%)",
                     m.scheduledTotal, m.scheduledOnTime, m.scheduledLate, m.scheduledFailed, m.scheduledRejected, m.rejectedRate * 100.0f);
        DES_LOG_INFO(log, "Background  : total=%d  completed=%d  failed=%d", m.backgroundTotal, m.backgroundCompleted, m.backgroundFailed);
        DES_LOG_INFO(log, "Interrupt   : total=%d  completed=%d", m.interruptTotal, m.interruptCompleted);
        DES_LOG_INFO(log, "Lateness    : avg=%.0fs  min=%ds  max=%ds  avg-early=%.0fs", m.avgLateness, m.minLateness, m.maxLateness, m.avgEarlyArrival);
        DES_LOG_INFO(log, "Time [h]    : idle=%.1f  moving=%.1f  searching=%.1f  accompany=%.1f  charging=%.1f  talk=%.1f",
                     m.idleTime / 3600.0, m.movingTime / 3600.0, m.searchingTime / 3600.0, m.accompanyTime / 3600.0, m.chargingTime / 3600.0, m.talkTime / 3600.0);
        DES_LOG_INFO(log, "Battery     : cycles=%d (full=%d, partial=%d)  deep-discharge=%d  avg-DoD=%.2f  equiv-cycles=%.1f  discharged=%.1fAh",
                     m.chargeCyclesTotal, m.chargeCyclesComplete, m.chargeCyclesPartial, m.deepDischargeCount, m.avgDepthOfDischarge, m.equivalentFullCycles, m.energyTotalConsumedAh);
        DES_LOG_INFO(log, "Movement    : distance=%.0fm", m.totalDistance);

        if (r.missions.byDay.empty()) {
            DES_LOG_INFO(log, "=====================================================");
            return;
        }

        int tCompl = 0, tOut = 0, tUnr = 0, tFind = 0;
        for (const auto& [day, byMode] : r.missions.byDay) {
            const auto s = scheduledOf(byMode);
            tCompl += s.onTime + s.late;
            tOut   += failCount(s, "person outside");
            tUnr   += failCount(s, "unreachable room");
            tFind  += failCount(s, "missed in building");
        }
        const int findDenTot = tCompl + tFind;
        DES_LOG_INFO(log, "Accompany   : findable-success=%.1f%%  (completed=%d, findable-miss=%d)  outside=%d  unreachable=%d",
                     findDenTot > 0 ? 100.0 * tCompl / findDenTot : 0.0, tCompl, tFind, tOut, tUnr);
        DES_LOG_INFO(log, "--- Accompany per day (find%% ignores outside + unreachable) ---");
        DES_LOG_INFO(log, "day  compl  fail  outside  unreach  findFail  raw%%  find%%");
        for (const auto& [day, byMode] : r.missions.byDay) {
            const auto s = scheduledOf(byMode);
            const int completed  = s.onTime + s.late;
            const int dispatched = completed + s.failed;
            const int findFail   = failCount(s, "missed in building");
            const int findDen    = completed + findFail;
            DES_LOG_INFO(log, "%3d  %5d  %4d  %7d  %7d  %8d  %4.0f%%  %5.0f%%",
                         day, completed, s.failed, failCount(s, "person outside"), failCount(s, "unreachable room"), findFail,
                         dispatched > 0 ? 100.0 * completed / dispatched : 0.0,
                         findDen > 0 ? 100.0 * completed / findDen : 0.0);
        }
        DES_LOG_INFO(log, "=====================================================");
    }

    Row runRow(const RunReport& r) const {
        Row fields;
        auto add = [&](const std::string& key, const auto& value) {
            std::ostringstream os;
            os << value;
            fields.emplace_back(key, os.str());
        };

        add("run_id", m_runId);
        add("run_index", m_runIndex);
        add("scenario", m_scenario);
        add("round", m_round);
        add("seed", m_config->seed);
        add("round_seed", m_roundSeed);
        add("terminated_reason", m_terminatedReason);

        add("idle_time", r.idleTime);
        add("moving_time", r.movingTime);
        add("searching_time", r.searchingTime);
        add("accompany_time", r.accompanyTime);
        add("charging_time", r.chargingTime);
        add("talk_time", r.talkTime);
        add("scheduled_total", r.scheduledTotal);
        add("scheduled_on_time", r.scheduledOnTime);
        add("scheduled_late", r.scheduledLate);
        add("scheduled_failed", r.scheduledFailed);
        add("scheduled_cancelled", r.scheduledCancelled);
        add("scheduled_rejected", r.scheduledRejected);
        add("background_total", r.backgroundTotal);
        add("background_completed", r.backgroundCompleted);
        add("background_failed", r.backgroundFailed);
        add("background_cancelled", r.backgroundCancelled);
        add("interrupt_total", r.interruptTotal);
        add("interrupt_completed", r.interruptCompleted);
        add("avg_early_arrival", r.avgEarlyArrival);
        add("avg_lateness", r.avgLateness);
        add("min_lateness", r.minLateness);
        add("max_lateness", r.maxLateness);
        add("rejected_rate", r.rejectedRate);
        add("total_distance", r.totalDistance);
        add("energy_total_consumed_ah", r.energyTotalConsumedAh);
        add("energy_idle_ah", r.energyIdleAh);
        add("energy_searching_ah", r.energySearchingAh);
        add("energy_accompany_ah", r.energyAccompanyAh);
        add("energy_talk_ah", r.energyTalkAh);
        add("energy_charging_ah", r.energyChargingAh);
        add("utilization", r.utilization);
        add("idle_percent", r.idlePercent);
        add("charging_percent", r.chargingPercent);
        add("charge_cycles_total", r.chargeCyclesTotal);
        add("charge_cycles_complete", r.chargeCyclesComplete);
        add("charge_cycles_partial", r.chargeCyclesPartial);
        add("deep_discharge_count", r.deepDischargeCount);
        add("equivalent_full_cycles", r.equivalentFullCycles);
        add("avg_depth_of_discharge", r.avgDepthOfDischarge);

        return fields;
    }

    std::vector<Row> dailyRows(const Reduced& r) const {
        std::set<int> dayIndices;
        for (const auto& [day, _] : r.days) {
            dayIndices.insert(day);
        }
        for (const auto& [day, _] : r.missions.byDay) {
            dayIndices.insert(day);
        }

        std::vector<Row> rows;
        for (const int day : dayIndices) {
            Row fields;
            auto add = [&](const std::string& key, const auto& value) {
                std::ostringstream os;
                os << value;
                fields.emplace_back(key, os.str());
            };

            const auto spanIt = r.days.find(day);
            const des::reduce::DaySpan span = spanIt == r.days.end() ? des::reduce::DaySpan{} : spanIt->second;
            const auto modeIt = r.missions.byDay.find(day);
            const std::map<des::ExecutionMode, des::reduce::MissionStats> empty;
            const auto& byMode = modeIt == r.missions.byDay.end() ? empty : modeIt->second;

            auto statsOf = [&](const des::ExecutionMode mode) {
                const auto it = byMode.find(mode);
                return it == byMode.end() ? des::reduce::MissionStats{} : it->second;
            };
            const auto sched = statsOf(des::ExecutionMode::SCHEDULED);
            const auto bg    = statsOf(des::ExecutionMode::BACKGROUND);
            const auto intr  = statsOf(des::ExecutionMode::INTERRUPT);

            const auto distIt   = r.drives.distanceByDay.find(day);
            const auto cyclesIt = r.chargeCyclesByDay.find(day);

            add("run_id", m_runId);
            add("scenario", m_scenario);
            add("round", m_round);
            add("seed", m_config->seed);
            add("round_seed", m_roundSeed);
            add("day", day);

            add("scheduled_on_time", sched.onTime);
            add("scheduled_late", sched.late);
            add("scheduled_failed", sched.failed);
            add("scheduled_cancelled", sched.cancelled);
            add("scheduled_rejected", sched.rejected);
            add("scheduled_fail_outside", failCount(sched, "person outside"));
            add("scheduled_fail_unreachable", failCount(sched, "unreachable room"));
            add("scheduled_fail_findable", failCount(sched, "missed in building"));

            add("background_completed", bg.onTime + bg.late);
            add("background_failed", bg.failed);
            add("background_cancelled", bg.cancelled);
            add("interrupt_completed", intr.onTime + intr.late);

            add("distance", distIt == r.drives.distanceByDay.end() ? 0.0 : distIt->second);
            add("energy_ah", span.energyAh);
            add("charge_cycles", cyclesIt == r.chargeCyclesByDay.end() ? 0 : cyclesIt->second);
            add("min_soc", span.minSoc);
            add("idle_time", span.idleTime);
            add("mission_time", span.missionTime);
            add("charging_time", span.chargingTime);
            add("total_time", span.totalTime);
            add("utilization", span.totalTime > 0 ? 100.0 * span.missionTime / span.totalTime : 0.0);

            rows.push_back(std::move(fields));
        }
        return rows;
    }

    static std::string csvEscape(const std::string& value) {
        if (value.find(',') == std::string::npos && value.find('"') == std::string::npos) {
            return value;
        }
        std::string escaped = "\"";
        for (const char c : value) {
            if (c == '"') {
                escaped += '"';
            }
            escaped += c;
        }
        escaped += '"';
        return escaped;
    }

    static std::string headerLine(const Row& fields) {
        std::string line;
        for (const auto& [key, _] : fields) {
            if (!line.empty()) {
                line += ",";
            }
            line += key;
        }
        return line;
    }

    static std::optional<std::string> existingHeader(const std::string& path) {
        std::ifstream in(path);
        std::string line;
        if (!in.is_open() || !std::getline(in, line)) {
            return std::nullopt;
        }
        return line;
    }

    void appendCsv(const std::string& path, const std::vector<Row>& rows) {
        if (path.empty() || rows.empty()) {
            return;
        }
        const auto log = rclcpp::get_logger("des.metrics.csv");

        std::set<std::string> seen;
        for (const auto& [key, _] : rows.front()) {
            if (!seen.insert(key).second) {
                DES_LOG_ERROR(log, "Duplicate CSV column '%s' in %s, refusing to write", key.c_str(), path.c_str());
                return;
            }
        }

        std::error_code ec;
        const auto parent = std::filesystem::path(path).parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent, ec);
        }

        const std::string header = headerLine(rows.front());
        const auto present = existingHeader(path);
        if (present.has_value() && present.value() != header) {
            DES_LOG_ERROR(log, "Header of %s does not match the current column set, refusing to append", path.c_str());
            return;
        }

        std::ofstream out(path, std::ios::app);
        if (!out.is_open()) {
            DES_LOG_WARN(log, "Could not open CSV file: %s", path.c_str());
            return;
        }
        if (!present.has_value()) {
            out << header << "\n";
        }
        for (const auto& fields : rows) {
            for (size_t i = 0; i < fields.size(); ++i) {
                out << (i ? "," : "") << csvEscape(fields[i].second);
            }
            out << "\n";
        }

        DES_LOG_INFO(log, "CSV written: %zu row(s), scenario=%s round=%d -> %s", rows.size(), m_scenario.c_str(), m_round, path.c_str());
        if (path == m_csvPath) {
            m_runIndex++;
        }
    }
};

}  // namespace des::metrics
