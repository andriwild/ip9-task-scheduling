#pragma once

#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "../observer/observer.h"
#include "event_system_msgs/msg/metrics_report.hpp"

class MetricsNode final : public rclcpp::Node, public IObserver {
    // Per-state-name buckets — populated dynamically as states show up via
    // TimelineStateChange's name field. Plugin-owned states (e.g. "clean",
    // "acquire") slot in automatically without code changes here.
    std::map<std::string, int>    timePerStateName;
    std::map<std::string, double> energyPerStateName;
    // Aggregated by structural category so we can compute the high-level
    // idle/charging/utilization percentages regardless of plugin states.
    std::map<des::RobotStateType, int>    timePerCategory;
    std::map<des::RobotStateType, double> energyPerCategory;

    // Mission stats are tracked per-execution-mode so the report can show
    // scheduled, background, and interrupt missions in separate sections —
    // each kind has a different notion of "success" (scheduled cares about
    // lateness, background cares about completion ratio, interrupt about
    // arrival count).
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
    // Lateness aggregates only make sense for scheduled missions (deadlines).
    int accMissionToLateTime  = 0;
    int accMissionToEarlyTime = 0;
    int minLateness           = 0;
    int maxLateness           = 0;
    bool hasLateMission       = false;
    double movedDistance      = 0.0;
    double lastSoc            = -1.0;
    double batteryCapacity    = 0.0;
    bool wasDriving           = false;
    // Mission-active time tracking — robot is "working" while any mission is
    // in flight (regardless of robot-state). Depth counter handles nested
    // interrupts; RejectMission completes without a prior Start so we guard
    // against going below zero.
    int missionActiveTime  = 0;
    int missionActiveStart = -1;
    int missionActiveDepth = 0;
    // Most recently seen state, kept verbatim so we can attribute the next
    // elapsed slice on the next transition.
    bool hasLastState = false;
    des::RobotStateType lastCategory{};
    std::string lastStateName;

    rclcpp::Publisher<event_system_msgs::msg::MetricsReport>::SharedPtr m_publisher;

public:
    explicit MetricsNode() : Node("metrics_node") {
        m_publisher = this->create_publisher<event_system_msgs::msg::MetricsReport>(
            "/metrics_report", rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local());
    }

    std::string getName() override {
        return "Metrics";
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
        missionActiveTime     = 0;
        missionActiveStart    = -1;
        missionActiveDepth    = 0;
        hasLastState          = false;
        lastStateName.clear();
    }


    void onEvent(const int time, const des::EventType type, const std::string& /*message*/, const bool isDriving, const bool /*isCharging*/, const std::string& /*color*/ = "", const int /*missionId*/ = -1) override {
        if(isDriving && !wasDriving) {
            wasDriving = true;
            lastTimeMoved = time;
        } else if(!isDriving && wasDriving){
           moveTime += time - lastTimeMoved;
            wasDriving = false;
        }

        if (type == des::EventType::MISSION_START) {
            if (missionActiveDepth == 0) {
                missionActiveStart = time;
            }
            missionActiveDepth++;
        } else if (type == des::EventType::MISSION_COMPLETE) {
            // Reject-path completes without a prior MissionStart; guard the depth.
            if (missionActiveDepth > 0) {
                missionActiveDepth--;
                if (missionActiveDepth == 0 && missionActiveStart >= 0) {
                    missionActiveTime += time - missionActiveStart;
                    missionActiveStart = -1;
                }
            }
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
                // timeDiff > 0 = strictly late. timeDiff == 0 covers both
                // "exactly on deadline" (scheduled) and "no deadline at all"
                // (background/interrupt where context maps missing deadline to now).
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

    void publishReport() {
        auto msg = event_system_msgs::msg::MetricsReport();

        // Named buckets — pull what the existing msg explicitly tracks.
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

        // Total observed sim time = sum of all named-state buckets (each
        // simulated second is attributed to exactly one state).
        double totalTime = 0.0;
        for (const auto& [_, t] : timePerStateName) totalTime += t;
        if (totalTime > 0) {
            const int idleCat     = timePerCategory[des::RobotStateType::IDLE];
            const int chargingCat = timePerCategory[des::RobotStateType::CHARGING];
            msg.utilization      = static_cast<float>(missionActiveTime / totalTime * 100.0);
            msg.idle_percent     = static_cast<float>(idleCat           / totalTime * 100.0);
            msg.charging_percent = static_cast<float>(chargingCat       / totalTime * 100.0);
        } else {
            msg.utilization      = 0.0f;
            msg.idle_percent     = 0.0f;
            msg.charging_percent = 0.0f;
        }

        RCLCPP_INFO(rclcpp::get_logger("Metrics"), "Publish Metrics");
        m_publisher->publish(msg);
        clear();
    }
};
