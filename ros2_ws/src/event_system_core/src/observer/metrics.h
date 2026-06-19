#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "../util/log.h"
#include "../util/types.h"
#include "../observer/observer.h"
#include "event_system_msgs/msg/metrics_report.hpp"

class MetricsNode final : public rclcpp::Node, public IObserver {
    std::map<std::string, int>    timePerStateName;
    std::map<std::string, double> energyPerStateName;
    std::map<des::RobotStateType, int>    timePerCategory;
    std::map<des::RobotStateType, double> energyPerCategory;

    struct MissionStats {
        int registered  = 0;  // queued / loaded into the sim
        int onTime      = 0;  // COMPLETED + timeDiff <= 0
        int late        = 0;  // COMPLETED + timeDiff > 0
        int failed      = 0;
        int cancelled   = 0;
        int rejected    = 0;
    };
    std::map<des::ExecutionMode, MissionStats> missionsByMode;

    int moveTime              = 0;
    int lastTimeStateChanged  = 0;
    int lastTimeMoved         = 0;
    int accMissionToLateTime  = 0;
    int accMissionToEarlyTime = 0;
    int minLateness           = 0;
    int maxLateness           = 0;
    bool hasLateMission       = false;
    double movedDistance      = 0.0;
    double lastSoc            = -1.0;
    double batteryCapacity    = 0.0;
    bool wasDriving           = false;

    bool hasLastState = false;
    des::RobotStateType lastCategory{};
    std::string lastStateName;

    int chargeCyclesTotal    = 0;
    int chargeCyclesComplete = 0;
    int deepDischargeCount   = 0;
    double dischargedAh      = 0.0;
    double dodSum            = 0.0;
    int dodCount             = 0;
    double lastChargeEndSoc  = -1.0;

    std::shared_ptr<const des::SimConfig> m_csvConfig;
    std::string m_csvPath;
    std::string m_csvScenario;
    int m_csvRound    = 0;
    int m_csvRunIndex = 0;

    rclcpp::Publisher<event_system_msgs::msg::MetricsReport>::SharedPtr m_publisher;

public:
    explicit MetricsNode() : Node("metrics_node") {
        m_publisher = this->create_publisher<event_system_msgs::msg::MetricsReport>(
            "/metrics_report", rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local());
    }

    std::string getName() override {
        return "Metrics";
    }

    void enableCsv(std::string path, std::shared_ptr<const des::SimConfig> config) {
        m_csvPath   = std::move(path);
        m_csvConfig = std::move(config);
    }

    void setRunInfo(const std::string& scenario, const int round) {
        m_csvScenario = scenario;
        m_csvRound    = round;
    }

    void clear() {
        timePerStateName.clear();
        energyPerStateName.clear();
        timePerCategory.clear();
        energyPerCategory.clear();
        moveTime              = 0;
        lastTimeStateChanged  = 0;
        lastTimeMoved         = 0;
        accMissionToLateTime  = 0;
        accMissionToEarlyTime = 0;
        minLateness           = 0;
        maxLateness           = 0;
        hasLateMission        = false;
        missionsByMode.clear();
        movedDistance         = 0.0;
        lastSoc               = -1.0;
        batteryCapacity       = 0.0;
        wasDriving            = false;
        hasLastState          = false;
        lastStateName.clear();
        chargeCyclesTotal     = 0;
        chargeCyclesComplete  = 0;
        deepDischargeCount    = 0;
        dischargedAh          = 0.0;
        dodSum                = 0.0;
        dodCount              = 0;
        lastChargeEndSoc      = -1.0;
    }


    void onEvent(const int time, const des::EventType type, const std::string& /*message*/, const bool isDriving, const bool /*isCharging*/, const std::string& /*color*/ = "", const int /*missionId*/ = -1) override {
        if(isDriving && !wasDriving) {
            wasDriving = true;
            lastTimeMoved = time;
        } else if(!isDriving && wasDriving){
           moveTime += time - lastTimeMoved;
            wasDriving = false;
        }

        if (type == des::EventType::CHARGE_MISSION_START) {
            chargeCyclesTotal++;
        }
        if (type == des::EventType::BATTERY_FULL || type == des::EventType::CHARGE_MISSION) {
            chargeCyclesComplete++;
        }

        if(type == des::EventType::SIMULATION_END){
            publishReport();
        }
    };

    void onStateChanged(const int time, const des::RobotStateType& newState, const std::string& newName, des::BatteryProps batStats) override {
        const int passedTime = time - lastTimeStateChanged;

        if (lastSoc < 0.0) {
            batteryCapacity = batStats.capacity;
        }
        const double consumedAh = (lastSoc < 0.0) ? 0.0 : (lastSoc - batStats.soc) * batteryCapacity;

        if (hasLastState) {
            timePerStateName[lastStateName]   += passedTime;
            energyPerStateName[lastStateName] += consumedAh;
            timePerCategory[lastCategory]     += passedTime;
            energyPerCategory[lastCategory]   += consumedAh;
        }

        if (consumedAh > 0.0) {
            dischargedAh += consumedAh;
        }
        const double lowFraction = batStats.lowThreshold / 100.0;
        if (lastSoc >= 0.0 && lastSoc >= lowFraction && batStats.soc < lowFraction) {
            deepDischargeCount++;
        }
        const bool wasCharging = lastStateName == "charging";
        const bool isCharging  = newName == "charging";
        if (!wasCharging && isCharging) {
            if (lastChargeEndSoc >= 0.0) {
                const double dod = lastChargeEndSoc - batStats.soc;
                if (dod > 0.0) { dodSum += dod; dodCount++; }
            }
        } else if (wasCharging && !isCharging) {
            lastChargeEndSoc = batStats.soc;
        }

        lastSoc              = batStats.soc;
        lastCategory         = newState;
        lastStateName        = newName;
        hasLastState         = true;
        lastTimeStateChanged = time;
    }

    void onMissionRegistered(const des::OrderPtr& order) override {
        if (!order) return;
        missionsByMode[order->execution].registered++;
    }

    void onMissionComplete(int /*time*/, const des::MissionState& state, const int timeDiff, des::ExecutionMode execution) override {
        auto& s = missionsByMode[execution];
        switch (state) {
            case des::MissionState::COMPLETED:
                if (timeDiff > 0) {
                    s.late++;
                    accMissionToLateTime += timeDiff;
                    if (!hasLateMission || timeDiff < minLateness) minLateness = timeDiff;
                    if (!hasLateMission || timeDiff > maxLateness) maxLateness = timeDiff;
                    hasLateMission = true;
                } else {
                    s.onTime++;
                    accMissionToEarlyTime += timeDiff;
                }
                break;
            case des::MissionState::CANCELLED: s.cancelled++; break;
            case des::MissionState::FAILED:    s.failed++;    break;
            case des::MissionState::REJECTED:  s.rejected++;  break;
            default: break;
        }
    }

    void onRobotMoved(int /*time*/, const std::string& /*location*/, double distance) override {
        movedDistance += distance;
    }

    bool hasData() const { return hasLastState; }

    event_system_msgs::msg::MetricsReport buildReport() {
        auto msg = event_system_msgs::msg::MetricsReport();

        auto timeFor   = [&](const std::string& n) { auto it = timePerStateName.find(n);   return it == timePerStateName.end()   ? 0    : it->second; };
        auto energyFor = [&](const std::string& n) { auto it = energyPerStateName.find(n); return it == energyPerStateName.end() ? 0.0  : it->second; };

        msg.idle_time          = timeFor("idle");
        msg.moving_time        = moveTime;
        msg.searching_time     = timeFor("search");
        msg.accompany_time     = timeFor("accompany");
        msg.charging_time      = timeFor("charging");
        msg.talk_time          = timeFor("conversate");

        auto stats = [&](des::ExecutionMode m) -> MissionStats {
            auto it = missionsByMode.find(m);
            return it == missionsByMode.end() ? MissionStats{} : it->second;
        };
        const auto sched  = stats(des::ExecutionMode::SCHEDULED);
        const auto bg     = stats(des::ExecutionMode::BACKGROUND);
        const auto intr   = stats(des::ExecutionMode::INTERRUPT);

        msg.scheduled_total     = sched.registered;
        msg.scheduled_on_time   = sched.onTime;
        msg.scheduled_late      = sched.late;
        msg.scheduled_failed    = sched.failed;
        msg.scheduled_cancelled = sched.cancelled;
        msg.scheduled_rejected  = sched.rejected;

        msg.background_total     = bg.registered;
        msg.background_completed = bg.onTime + bg.late;
        msg.background_failed    = bg.failed;
        msg.background_cancelled = bg.cancelled;

        msg.interrupt_total     = intr.registered;
        msg.interrupt_completed = intr.onTime + intr.late;

        // Lateness aggregates apply to scheduled (only those have deadlines).
        msg.avg_early_arrival = (sched.onTime > 0) ? (float)accMissionToEarlyTime / sched.onTime : 0.0f;
        msg.avg_lateness      = (sched.late > 0)   ? (float)accMissionToLateTime  / sched.late   : 0.0f;
        msg.min_lateness      = hasLateMission ? minLateness : 0;
        msg.max_lateness      = hasLateMission ? maxLateness : 0;

        const int schedTotal = sched.registered;
        msg.rejected_rate = (schedTotal > 0) ? static_cast<float>(sched.rejected) / schedTotal : 0.0f;

        msg.energy_idle_ah       = static_cast<float>(energyFor("idle"));
        msg.energy_searching_ah  = static_cast<float>(energyFor("search"));
        msg.energy_accompany_ah  = static_cast<float>(energyFor("accompany"));
        msg.energy_talk_ah       = static_cast<float>(energyFor("conversate"));
        msg.energy_charging_ah   = static_cast<float>(energyFor("charging"));
        double energyTotal = 0.0;
        for (const auto& [_, e] : energyPerStateName) energyTotal += e;
        msg.energy_total_consumed_ah = static_cast<float>(energyTotal);

        msg.total_distance = static_cast<float>(movedDistance);

        // Total observed sim time = sum of all named-state buckets
        double totalTime = 0.0;
        for (const auto& [_, t] : timePerStateName) totalTime += t;
        if (totalTime > 0) {
            const int missionCat   = timePerCategory[des::RobotStateType::MISSION];
            const int chargingCat  = timePerCategory[des::RobotStateType::CHARGING];
            const int returningName = timeFor("returning");
            const int idleName      = timeFor("idle");
            msg.utilization       = static_cast<float>(missionCat    / totalTime * 100.0);
            msg.idle_percent      = static_cast<float>(idleName      / totalTime * 100.0);
            msg.charging_percent  = static_cast<float>(chargingCat   / totalTime * 100.0);
            msg.returning_percent = static_cast<float>(returningName / totalTime * 100.0);
        } else {
            msg.utilization       = 0.0f;
            msg.idle_percent      = 0.0f;
            msg.charging_percent  = 0.0f;
            msg.returning_percent = 0.0f;
        }

        msg.charge_cycles_total    = chargeCyclesTotal;
        msg.charge_cycles_complete = chargeCyclesComplete;
        msg.charge_cycles_partial  = std::max(0, chargeCyclesTotal - chargeCyclesComplete);
        msg.deep_discharge_count   = deepDischargeCount;
        msg.equivalent_full_cycles = (batteryCapacity > 0.0) ? static_cast<float>(dischargedAh / batteryCapacity) : 0.0f;
        msg.avg_depth_of_discharge = (dodCount > 0) ? static_cast<float>(dodSum / dodCount) : 0.0f;

        return msg;
    }

    void publishReport() {
        const auto msg = buildReport();
        DES_LOG_INFO(rclcpp::get_logger("des.observer.metrics"), "Publish Metrics");
        DES_LOG_INFO(rclcpp::get_logger("des.observer.metrics"),
                     "Battery: cycles=%d (full=%d, partial=%d), chargingTime=%ds, deepDischarges=%d, equivFullCycles=%.2f, avgDoD=%.2f",
                     msg.charge_cycles_total, msg.charge_cycles_complete, msg.charge_cycles_partial,
                     msg.charging_time, msg.deep_discharge_count, msg.equivalent_full_cycles, msg.avg_depth_of_discharge);
        if (!m_csvPath.empty() && m_csvConfig && hasLastState) {
            writeCsvRow(msg);
        }
        m_publisher->publish(msg);
        clear();
    }

private:
    // Appends one CSV row per finished simulation: run metadata + config
    // parameters + outcome metrics. Enabled via enableCsv() (headless only).
    void writeCsvRow(const event_system_msgs::msg::MetricsReport& r) {
        std::vector<std::pair<std::string, std::string>> fields;
        auto add = [&](const std::string& key, const auto& value) {
            std::ostringstream os;
            os << value;
            fields.emplace_back(key, os.str());
        };

        add("run_index", m_csvRunIndex);
        add("scenario", m_csvScenario);
        add("round", m_csvRound);

        add("always_charge_at_dock", m_csvConfig->alwaysChargeAtDock ? 1 : 0);
        add("charge_to_full", m_csvConfig->chargeToFull ? 1 : 0);
        add("cv_threshold", m_csvConfig->cvThreshold);
        add("taper_fraction", m_csvConfig->taperFraction);
        add("battery_voltage", m_csvConfig->batteryVoltage);
        add("charging_rate", m_csvConfig->chargingRate);
        add("low_battery_threshold", m_csvConfig->lowBatteryThreshold);
        add("full_battery_threshold", m_csvConfig->fullBatteryThreshold);
        add("battery_capacity", m_csvConfig->batteryCapacity);
        add("initial_battery_capacity", m_csvConfig->initialBatteryCapacity);
        add("energy_consumption_base", m_csvConfig->energyConsumptionBase);
        add("energy_consumption_drive", m_csvConfig->energyConsumptionDrive);
        add("robot_speed", m_csvConfig->robotSpeed);

        add("idle_time", r.idle_time);
        add("moving_time", r.moving_time);
        add("searching_time", r.searching_time);
        add("accompany_time", r.accompany_time);
        add("charging_time", r.charging_time);
        add("talk_time", r.talk_time);
        add("scheduled_total", r.scheduled_total);
        add("scheduled_on_time", r.scheduled_on_time);
        add("scheduled_late", r.scheduled_late);
        add("scheduled_failed", r.scheduled_failed);
        add("scheduled_cancelled", r.scheduled_cancelled);
        add("scheduled_rejected", r.scheduled_rejected);
        add("background_total", r.background_total);
        add("background_completed", r.background_completed);
        add("background_failed", r.background_failed);
        add("background_cancelled", r.background_cancelled);
        add("interrupt_total", r.interrupt_total);
        add("interrupt_completed", r.interrupt_completed);
        add("avg_early_arrival", r.avg_early_arrival);
        add("avg_lateness", r.avg_lateness);
        add("min_lateness", r.min_lateness);
        add("max_lateness", r.max_lateness);
        add("rejected_rate", r.rejected_rate);
        add("total_distance", r.total_distance);
        add("energy_total_consumed_ah", r.energy_total_consumed_ah);
        add("energy_idle_ah", r.energy_idle_ah);
        add("energy_searching_ah", r.energy_searching_ah);
        add("energy_accompany_ah", r.energy_accompany_ah);
        add("energy_talk_ah", r.energy_talk_ah);
        add("energy_charging_ah", r.energy_charging_ah);
        add("utilization", r.utilization);
        add("idle_percent", r.idle_percent);
        add("charging_percent", r.charging_percent);
        add("returning_percent", r.returning_percent);
        add("charge_cycles_total", r.charge_cycles_total);
        add("charge_cycles_complete", r.charge_cycles_complete);
        add("charge_cycles_partial", r.charge_cycles_partial);
        add("deep_discharge_count", r.deep_discharge_count);
        add("equivalent_full_cycles", r.equivalent_full_cycles);
        add("avg_depth_of_discharge", r.avg_depth_of_discharge);

        std::error_code ec;
        const auto parent = std::filesystem::path(m_csvPath).parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent, ec);
        }
        const bool writeHeader = !std::filesystem::exists(m_csvPath) || std::filesystem::file_size(m_csvPath, ec) == 0;

        std::ofstream out(m_csvPath, std::ios::app);
        if (!out.is_open()) {
            DES_LOG_WARN(rclcpp::get_logger("des.observer.metrics"), "Could not open CSV file: %s", m_csvPath.c_str());
            return;
        }
        if (writeHeader) {
            for (size_t i = 0; i < fields.size(); ++i) {
                out << (i ? "," : "") << fields[i].first;
            }
            out << "\n";
        }
        for (size_t i = 0; i < fields.size(); ++i) {
            out << (i ? "," : "") << fields[i].second;
        }
        out << "\n";

        DES_LOG_INFO(rclcpp::get_logger("des.observer.metrics"), "CSV row written: scenario=%s round=%d -> %s", m_csvScenario.c_str(), m_csvRound, m_csvPath.c_str());
        m_csvRunIndex++;
    }
};
