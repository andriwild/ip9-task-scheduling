#pragma once

#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "observer.h"
#include "../model/i_sim_context.h"
#include "../model/person.h"
#include "../model/robot.h"
#include "../plugins/order_registry.h"
#include "../plugins/accompany/accompany_order.h"
#include "../util/log.h"
#include "../util/types.h"

class MissionTraceObserver final : public IObserver {
public:
    MissionTraceObserver(
        const ISimContext* ctx,
        des::RoomMap rooms,
        std::string outputPath,
        std::shared_ptr<const des::SimConfig> config = nullptr)
        : m_ctx(ctx)
        , m_rooms(std::move(rooms))
        , m_outputPath(std::move(outputPath))
    {
        if (config) {
            m_enabledRounds.assign(config->missionTraceRounds.begin(), config->missionTraceRounds.end());
            if (config->missionTraceWindow.size() == 2) {
                m_windowFrom = config->missionTraceWindow[0];
                m_windowTo   = config->missionTraceWindow[1];
            }
        }
    }

    std::string getName() override { return "MissionTrace"; }

    void onRobotMoved(const int time, const std::string& location, const double /*distance*/) override {
        if (!recording(time)) {
            return;
        }
        auto* trace = active();
        if (!trace) {
            return;
        }
        appendRobot(*trace, time, location, "move");
        snapshotPerson(*trace, time);
    }

    void onRobotMovedTo(const int time, const des::Point& position, double /*distance*/ = 0.0) override {
        if (!recording(time)) {
            return;
        }
        auto* trace = active();
        if (!trace) {
            return;
        }
        const std::string event = m_ctx->getRobot()->getState()->getName() == "search" ? "scan" : "move";
        trace->robot.push_back(Pt{ time, m_ctx->getRobot()->getLocation(), position.m_x, position.m_y, event });
    }

    void onStateChanged(const int time, const des::RobotStateType& /*type*/, const std::string& name, des::BatteryProps /*bat*/) override {
        if (!recording(time)) {
            return;
        }
        auto* trace = active();
        if (!trace) {
            return;
        }
        if (name != trace->lastRobotState) {
            trace->lastRobotState = name;
            appendRobot(*trace, time, m_ctx->getRobot()->getLocation(), "state:" + name);
        }
    }

    void onEvent(const int time, const des::EventType type, const std::string& /*message*/, const bool /*isDriving*/, const bool /*isCharging*/, const std::string& /*color*/ = "", const int missionId = -1) override {
        if (type == des::EventType::SIMULATION_END) {
            snapshotAllPersons(time);
            flush();
            return;
        }
        if (!recording(time)) {
            return;
        }
        if (isPersonEvent(type)) {
            snapshotAllPersons(time);
            if (auto* trace = active()) {
                snapshotPerson(*trace, time);
            }
        }
        if (missionId >= 0 && type == des::EventType::ABORT_SEARCH) {
            if (auto it = m_traces.find(missionId); it != m_traces.end()) {
                appendRobot(it->second, time, m_ctx->getRobot()->getLocation(), "abort");
            }
        }
    }

    void onMissionComplete(const int /*time*/, const des::OrderPtr& order, const int /*timeDiff*/) override {
        if (!order) {
            return;
        }
        auto it = m_traces.find(order->id);
        if (it == m_traces.end()) {
            return;
        }
        it->second.outcome = des::missionStateStr(order->state);
        const std::string detail = OrderRegistry::instance().get(order->type).outcomeDetail(*order);
        if (!detail.empty()) {
            it->second.outcome += " (" + detail + ")";
        }
    }

private:
    bool recording(const int time) const {
        if (time < m_windowFrom || time > m_windowTo) {
            return false;
        }
        if (m_enabledRounds.empty()) {
            return true;
        }
        return std::find(m_enabledRounds.begin(), m_enabledRounds.end(), m_flushCount + 1) != m_enabledRounds.end();
    }

    struct Pt {
        int time;
        std::string location;
        std::optional<double> x, y;
        std::string event;
    };

    struct Trace {
        int id = 0;
        std::string type;
        std::string personName;
        std::string room;
        std::optional<int> deadline;
        std::string outcome;
        std::vector<Pt> robot;
        std::vector<Pt> personPath;
        std::string lastPersonLoc;
        std::string lastRobotState;
    };

    struct PersonTrack {
        std::string color;
        std::string workplace;
        std::string lastLoc;
        std::vector<Pt> path;
    };

    Trace* active() {
        const auto order = m_ctx->getOrderPtr();
        if (!order) {
            return nullptr;
        }
        auto [it, inserted] = m_traces.try_emplace(order->id);
        Trace& trace = it->second;
        if (inserted) {
            trace.id = order->id;
            trace.type = order->type;
            trace.deadline = order->deadline;
            if (const auto target = OrderRegistry::instance().get(order->type).targetLocation(*order)) {
                trace.room = *target;
            }
            if (order->type == "accompany") {
                trace.personName = static_cast<const AccompanyOrder&>(*order).personName;
                snapshotPerson(trace, m_ctx->getTime());
            }
        }
        return &trace;
    }

    std::optional<std::pair<double, double>> coordsOf(const std::string& location) const {
        const auto it = m_rooms.find(location);
        if (it == m_rooms.end()) {
            return std::nullopt;
        }
        return std::make_pair(it->second.m_p.m_x, it->second.m_p.m_y);
    }

    void appendRobot(Trace& trace, const int time, const std::string& location, const std::string& event) {
        Pt pt{ time, location, std::nullopt, std::nullopt, event };
        if (const auto c = coordsOf(location)) {
            pt.x = c->first;
            pt.y = c->second;
        }
        trace.robot.push_back(std::move(pt));
    }

    void snapshotPerson(Trace& trace, const int time) {
        if (trace.personName.empty()) {
            return;
        }
        const auto& locations = m_ctx->getAllPersonLocations();
        const auto it = locations.find(trace.personName);
        if (it == locations.end() || it->second == trace.lastPersonLoc) {
            return;
        }
        trace.lastPersonLoc = it->second;
        Pt pt{ time, it->second, std::nullopt, std::nullopt, "" };
        if (const auto pos = m_ctx->getPersonPosition(trace.personName)) {
            pt.x = pos->m_x;
            pt.y = pos->m_y;
        }
        trace.personPath.push_back(std::move(pt));
    }

    void snapshotAllPersons(const int time) {
        for (const auto& [name, loc] : m_ctx->getAllPersonLocations()) {
            auto [it, inserted] = m_persons.try_emplace(name);
            PersonTrack& track = it->second;
            if (inserted) {
                if (const auto person = m_ctx->getPersonByName(name)) {
                    track.color = person->color;
                    track.workplace = person->workplace;
                }
            }
            if (loc == track.lastLoc) {
                continue;
            }
            track.lastLoc = loc;
            Pt pt{ time, loc, std::nullopt, std::nullopt, "" };
            if (const auto pos = m_ctx->getPersonPosition(name)) {
                pt.x = pos->m_x;
                pt.y = pos->m_y;
            }
            track.path.push_back(std::move(pt));
        }
    }

    static bool isPersonEvent(const des::EventType type) {
        switch (type) {
            case des::EventType::PERSON_TRANSITION:
            case des::EventType::PERSON_ARRIVED:
            case des::EventType::PERSON_DEPARTURE:
            case des::EventType::PERSON_ROOM_ARRIVED:
            case des::EventType::PERSON_ACCOMPANY_DEPARTURE:
            case des::EventType::PERSON_ACCOMPANY_ARRIVED:
                return true;
            default:
                return false;
        }
    }

    nlohmann::json sightingsToJson() const {
        nlohmann::json out;
        out["rooms"] = nlohmann::json::array();
        out["byPerson"] = nlohmann::json::object();
        if (!m_ctx || !m_ctx->getRobot()) {
            return out;
        }

        std::vector<std::string> rooms;
        std::map<std::string, int> roomIndex;
        for (const auto& s : m_ctx->getRobot()->getSightings().entries()) {
            auto [it, inserted] = roomIndex.try_emplace(s.location, static_cast<int>(rooms.size()));
            if (inserted) {
                rooms.push_back(s.location);
            }
            if (!out["byPerson"].contains(s.personName)) {
                out["byPerson"][s.personName] = nlohmann::json::array();
            }
            out["byPerson"][s.personName].push_back(nlohmann::json::array({
                s.time, it->second, s.kind == SightingKind::PRESENT ? 1 : 0,
            }));
        }
        out["rooms"] = std::move(rooms);
        return out;
    }

    static nlohmann::json stepsToJson(const std::vector<Pt>& steps) {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& s : steps) {
            nlohmann::json j;
            j["t"] = s.time;
            j["loc"] = s.location;
            if (s.x) {
                j["x"] = *s.x;
            }
            if (s.y) {
                j["y"] = *s.y;
            }
            if (!s.event.empty()) {
                j["event"] = s.event;
            }
            arr.push_back(std::move(j));
        }
        return arr;
    }

    std::string pathForFlush() const {
        if (m_flushCount == 0) {
            return m_outputPath;
        }
        const auto dot = m_outputPath.rfind(".json");
        const std::string base = dot == std::string::npos ? m_outputPath : m_outputPath.substr(0, dot);
        return base + "_r" + std::to_string(m_flushCount + 1) + ".json";
    }

    void flush() {
        if (!m_outputPath.empty() && (!m_traces.empty() || !m_persons.empty())) {
            nlohmann::json missions = nlohmann::json::array();
            for (const auto& [id, trace] : m_traces) {
                nlohmann::json m;
                m["missionId"] = trace.id;
                m["type"] = trace.type;
                if (!trace.personName.empty()) {
                    m["person"] = trace.personName;
                }
                if (!trace.room.empty()) {
                    m["appointmentRoom"] = trace.room;
                }
                if (trace.deadline) {
                    m["deadline"] = *trace.deadline;
                }
                m["outcome"] = trace.outcome;
                m["robot"] = stepsToJson(trace.robot);
                m["person_path"] = stepsToJson(trace.personPath);
                missions.push_back(std::move(m));
            }

            nlohmann::json persons = nlohmann::json::array();
            for (const auto& [name, track] : m_persons) {
                if (track.path.empty()) {
                    continue;
                }
                nlohmann::json p;
                p["name"] = name;
                if (!track.color.empty()) {
                    p["color"] = track.color;
                }
                if (!track.workplace.empty()) {
                    p["workplace"] = track.workplace;
                }
                p["path"] = stepsToJson(track.path);
                persons.push_back(std::move(p));
            }

            nlohmann::json root;
            root["missions"] = std::move(missions);
            root["persons"] = std::move(persons);
            root["sightings"] = sightingsToJson();
            root["meta"] = {
                { "search_reward_strategy", des::searchRewardStrategyToString(m_ctx->getConfig()->searchRewardStrategy) },
            };

            const std::string path = pathForFlush();
            std::ofstream out(path);
            if (out.is_open()) {
                out << root.dump(2);
                DES_LOG_INFO(rclcpp::get_logger("des.observer.trace"), "Mission trace written: %zu missions, %zu persons -> %s", m_traces.size(), m_persons.size(), path.c_str());
            } else {
                DES_LOG_ERROR(rclcpp::get_logger("des.observer.trace"), "Could not write mission trace: %s", path.c_str());
            }
        }
        m_traces.clear();
        m_persons.clear();
        m_flushCount++;
    }

    const ISimContext* m_ctx;
    des::RoomMap m_rooms;
    std::string m_outputPath;
    std::map<int, Trace> m_traces;
    std::map<std::string, PersonTrack> m_persons;
    int m_flushCount = 0;
    std::vector<int> m_enabledRounds;
    int m_windowFrom = std::numeric_limits<int>::min();
    int m_windowTo   = std::numeric_limits<int>::max();
};
