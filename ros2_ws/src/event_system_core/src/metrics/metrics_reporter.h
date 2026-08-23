#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <format>
#include <iostream>
#include <functional>
#include <ranges>
#include <string>
#include <utility>

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "plugins/accompany/events/scan_point_event.h"
#include "plugins/charge/charge_order.h"
#include "plugins/order_registry.h"
#include "util/constants.h"
#include "metrics/debug_trace.h"
#include "util/types.h"

namespace des::metrics {

class MetricsReporter {
    std::string m_csvPath;
    std::string m_dailyCsvPath;
    std::string m_scenario;
    unsigned int m_roundSeed = 0;
    std::string m_debugDir;
    int m_roundsWritten = 0;

public:
    void enableCsv(std::string path) {
        m_csvPath = std::move(path);
    }

    void enableDailyCsv(std::string path) {
        m_dailyCsvPath = std::move(path);
    }

    void enableDebugTrace(std::string directory) {
        m_debugDir = std::move(directory);
    }

    void setRunInfo(std::string scenario, const unsigned int roundSeed) {
        m_scenario  = std::move(scenario);
        m_roundSeed = roundSeed;
    }

    void report(const ISimContext& ctx, const EventList& protocol) {
        using std::views::filter;
        using std::views::transform;
        using std::views::chunk_by;

        const auto& states = ctx.getRobot()->getStateLog().entries();
        auto secondsIn = [&states](const std::string& stateName) {
            return std::ranges::fold_left(
                states | filter([&](const StateInterval& s) { return s.name == stateName; })
                       | transform([](const StateInterval& s) { return s.endTime - s.time; }),
                0, std::plus{});
        };

        auto scheduled = protocol
            | filter([](const auto& e) { return e->getType() == EventType::MISSION_COMPLETE; })
            | filter([](const auto& e) { return e->getOrder() && e->getOrder()->execution == ExecutionMode::SCHEDULED; });

        auto countIn = [&protocol](const ExecutionMode mode, const OrderState state) {
            return std::ranges::count_if(protocol, [mode, state](const auto& e) {
                const auto& order = e->getOrder();
                return e->getType() == EventType::MISSION_COMPLETE
                    && order
                    && order->execution == mode
                    && order->state == state
                    && order->type != kChargeOrderType;
            });
        };

        auto countType = [&protocol](const std::string& type) {
            return std::ranges::count_if(protocol, [&type](const auto& e) {
                const auto& order = e->getOrder();
                return e->getType() == EventType::MISSION_COMPLETE
                    && order
                    && order->execution == ExecutionMode::BACKGROUND
                    && order->state == OrderState::COMPLETED
                    && order->type == type;
            });
        };

        auto inState = [](const OrderState state) {
            return [state](const auto& e) { return e->getOrder()->state == state; };
        };

        auto withDetail = [](const std::string& detail) {
            return [detail](const auto& e) {
                return OrderRegistry::instance().get(e->getOrder()->type).outcomeDetail(*e->getOrder()) == detail;
            };
        };

        // create chunks of missions by checking if two following events are from the same day
        auto sameDay = [](const auto& a, const auto& b) {
            return a->time / SECONDS_PER_DAY == b->time / SECONDS_PER_DAY;
        };

        auto scansOn = [&protocol](const int day) {
            return std::ranges::count_if(protocol, [day](const auto& e) {
                return e->getType() == EventType::SCAN && e->time / SECONDS_PER_DAY == day;
            });
        };

        // A room visit is a run of scans in the same room within the same mission.
        // Detour stops stay inside the room, a room entered again later counts again.
        auto roomsOn = [&protocol](const int day) {
            int rooms = 0;
            int lastMission = -1;
            std::string lastRoom;
            for (const auto& event : protocol) {
                if (event->getType() != EventType::SCAN || event->time / SECONDS_PER_DAY != day) {
                    continue;
                }
                const auto* scan = dynamic_cast<const ScanPointEvent*>(event.get());
                if (scan == nullptr) {
                    continue;
                }
                if (scan->getMissionId() != lastMission || scan->m_room != lastRoom) {
                    ++rooms;
                    lastMission = scan->getMissionId();
                    lastRoom    = scan->m_room;
                }
            }
            return rooms;
        };

        auto searchOn = [&states](const int day) {
            auto intervals = states | filter([day](const StateInterval& s) {
                return s.name == "search" && s.time / SECONDS_PER_DAY == day;
            });
            const int seconds = std::ranges::fold_left(
                intervals | transform([](const StateInterval& s) { return s.endTime - s.time; }),
                0, std::plus{});
            const double metres = std::ranges::fold_left(
                intervals | transform([](const StateInterval& s) { return s.distTo - s.distFrom; }),
                0.0, std::plus{});
            return std::pair{ seconds, metres };
        };

        const bool firstRound = m_roundsWritten == 0;
        auto write = [firstRound](const std::string& path, const std::string& header, const std::string& rows) {
            std::error_code ec;
            std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
            std::ofstream out(path, firstRound ? std::ios::trunc : std::ios::app);
            if (firstRound) {
                out << header;
                std::cout << header;
            }
            out << rows;
            std::cout << rows;
        };

        auto fullCycles = ctx.getRobot()->getDischargedAh() / ctx.getRobot()->batteryStats().capacity;

        const auto& sessions = ctx.getRobot()->getChargeSessions();
        const int chargeStarts = static_cast<int>(sessions.size());
        const double minSoc = std::ranges::fold_left(
            sessions | transform([](const ChargeSession& s) { return s.soc; }),
            ctx.getRobot()->batteryStats().soc,
            [](const double a, const double b) { return std::min(a, b); });

        auto chargesWith = [&sessions](const ChargeTrigger trigger) {
            return std::ranges::count_if(sessions, [trigger](const ChargeSession& s) {
                return s.trigger == trigger;
            });
        };

        int onTime = 0;
        int late = 0;
        long lateSum = 0;
        long earlySum = 0;
        for (const auto& e : scheduled) {
            const auto& order = e->getOrder();
            if (order->state != OrderState::COMPLETED || !order->deadline.has_value()) {
                continue;
            }
            const int delta = e->time - *order->deadline;
            if (delta > 0) {
                ++late;
                lateSum += delta;
            } else {
                ++onTime;
                earlySum -= delta;
            }
        }
        const double lateMean = late > 0 ? static_cast<double>(lateSum) / late : 0.0;
        const double earlyMean = onTime > 0 ? static_cast<double>(earlySum) / onTime : 0.0;

        std::string daily;
        for (auto day : scheduled | chunk_by(sameDay)) {
            const int dayIndex = day.front()->time / SECONDS_PER_DAY;
            const auto [searchSeconds, searchMetres] = searchOn(dayIndex);
            daily += std::format("{},{},{},{},{},{},{},{},{},{},{:g}\n",
                m_roundSeed, m_scenario,
                dayIndex,
                std::ranges::count_if(day, inState(OrderState::COMPLETED)),
                std::ranges::count_if(day, inState(OrderState::FAILED)),
                std::ranges::count_if(day, withDetail("missed in building")),
                std::ranges::count_if(day, inState(OrderState::REJECTED)),
                scansOn(dayIndex), roomsOn(dayIndex), searchSeconds, searchMetres);
        }
        write(m_dailyCsvPath, "seed,scenario,day,completed,failed,findable_miss,rejected,scans,rooms,search_s,search_m\n", daily);

        const std::string run = std::format(
            "{},{},{},{},{},{},{},{:g},{},{:g},{},{:g},{},{},{},{},{},{:g},{:g},{},{},{},{},{},{}\n",
            m_roundSeed, m_scenario,
            secondsIn("idle"), secondsIn("search"), secondsIn("accompany"),
            secondsIn("charging"), secondsIn("conversate"),
            ctx.getRobot()->getOdometer(),
            std::ranges::distance(scheduled),
            fullCycles,
            chargeStarts,
            minSoc,
            chargesWith(ChargeTrigger::PLANNED),
            chargesWith(ChargeTrigger::REACTIVE),
            chargesWith(ChargeTrigger::OPPORTUNISTIC),
            onTime,
            late,
            lateMean,
            earlyMean,
            countIn(ExecutionMode::SCHEDULED, OrderState::COMPLETED),
            countIn(ExecutionMode::BACKGROUND, OrderState::COMPLETED),
            countIn(ExecutionMode::INTERRUPT, OrderState::COMPLETED),
            countIn(ExecutionMode::INTERRUPT, OrderState::REJECTED),
            countType("clean"),
            countType("data_acquisition"));
        write(m_csvPath, "seed,scenario,idle_s,search_s,accompany_s,charging_s,talk_s,distance_m,missions,full_cycles,charge_starts,min_soc,charge_planned,charge_reactive,charge_opportunistic,on_time,late,late_mean_s,early_mean_s,done_scheduled,done_background,done_interrupt,rejected_interrupt,done_clean,done_acquire\n", run);

        if (!m_debugDir.empty()) {
            const std::string dir = m_debugDir.back() == '/' ? m_debugDir : m_debugDir + "/";
            writeMoveTrace(dir + "moves.csv", protocol);
            writeEventTrace(dir + "events.csv", protocol);
            writeStateTrace(dir + "states.csv", ctx.getRobot()->getStateLog());
            writeSightingTrace(dir + "sightings.csv", ctx.getRobot()->getSightings());
            writeKnowledgeTrace(dir + "knowledge.csv", ctx.getRobot()->getSightings());
        }

        ++m_roundsWritten;
    }
};

}  // namespace des::metrics

