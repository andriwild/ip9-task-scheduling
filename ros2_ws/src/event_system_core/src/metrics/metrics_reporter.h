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
#include "plugins/order_registry.h"
#include "util/constants.h"
#include "util/types.h"

namespace des::metrics {

class MetricsReporter {
    std::string m_csvPath;
    std::string m_dailyCsvPath;
    std::string m_scenario;
    unsigned int m_roundSeed = 0;

public:
    void enableCsv(std::string path) {
        m_csvPath = std::move(path);
    }

    void enableDailyCsv(std::string path) {
        m_dailyCsvPath = std::move(path);
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
                states | filter([&](const des::StateInterval& s) { return s.name == stateName; })
                       | transform([](const des::StateInterval& s) { return s.endTime - s.time; }),
                0, std::plus{});
        };

        auto scheduled = protocol
            | filter([](const auto& e) { return e->getType() == des::EventType::MISSION_COMPLETE; })
            | filter([](const auto& e) { return e->getOrder() && e->getOrder()->execution == des::ExecutionMode::SCHEDULED; });

        auto inState = [](const des::MissionState state) {
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

        auto write = [](const std::string& path, const std::string& text) {
            std::error_code ec;
            std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
            std::ofstream(path) << text;
            std::cout << text;
        };

        std::string daily = "seed,scenario,day,completed,failed,findable_miss,rejected\n";
        for (auto day : scheduled | chunk_by(sameDay)) {
            daily += std::format("{},{},{},{},{},{},{}\n",
                m_roundSeed, m_scenario,
                day.front()->time / SECONDS_PER_DAY,
                std::ranges::count_if(day, inState(des::MissionState::COMPLETED)),
                std::ranges::count_if(day, inState(des::MissionState::FAILED)),
                std::ranges::count_if(day, withDetail("missed in building")),
                std::ranges::count_if(day, inState(des::MissionState::REJECTED)));
        }
        write(m_dailyCsvPath, daily);

        const std::string run = std::format(
            "seed,scenario,idle_s,search_s,accompany_s,charging_s,talk_s,distance_m,missions\n"
            "{},{},{},{},{},{},{},{:g},{}\n",
            m_roundSeed, m_scenario,
            secondsIn("idle"), secondsIn("search"), secondsIn("accompany"),
            secondsIn("charging"), secondsIn("conversate"),
            ctx.getRobot()->getOdometer(),
            std::ranges::distance(scheduled));
        write(m_csvPath, run);
    }
};

}  // namespace des::metrics
