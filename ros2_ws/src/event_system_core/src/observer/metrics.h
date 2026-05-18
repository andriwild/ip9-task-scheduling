#pragma once

#include <rclcpp/rclcpp.hpp>

#include "../observer/observer.h"
#include "event_system_msgs/msg/metrics_report.hpp"

class MetricsNode final : public rclcpp::Node, public IObserver {
    int searchTime            = 0;
    int accompanyTime         = 0;
    int idleTime              = 0;
    int moveTime              = 0;
    int talkTime              = 0;
    int chargingTime          = 0;
    int returningTime         = 0;
    int lastTimeStateChanged  = 0;
    int lastTimeMoved         = 0;
    int accMissionToLateTime  = 0;
    int accMissionToEarlyTime = 0;
    int nMissionCompleted     = 0;
    int nMissionFailed        = 0;
    int nMissionCompletedLate = 0;
    int nMissionCanceled      = 0;
    int nMissionRejected      = 0;
    int minLateness           = 0;
    int maxLateness           = 0;
    bool hasLateMission       = false;
    double movedDistance      = 0.0;
    double lastSoc            = -1.0;
    double batteryCapacity    = 0.0;
    double energyIdle         = 0.0;
    double energySearching    = 0.0;
    double energyAccompany    = 0.0;
    double energyTalk         = 0.0;
    double energyCharging     = 0.0;
    double energyReturning    = 0.0;
    bool wasDriving           = false;
    des::RobotStateType lastState;

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
        searchTime            = 0;
        accompanyTime         = 0;
        idleTime              = 0;
        moveTime              = 0;
        talkTime              = 0;
        chargingTime          = 0;
        returningTime         = 0;
        lastTimeStateChanged  = 0;
        lastTimeMoved         = 0;
        accMissionToLateTime  = 0;
        accMissionToEarlyTime = 0;
        nMissionCompleted     = 0;
        nMissionFailed        = 0;
        nMissionCompletedLate = 0;
        nMissionCanceled      = 0;
        nMissionRejected      = 0;
        minLateness           = 0;
        maxLateness           = 0;
        hasLateMission        = false;
        movedDistance         = 0.0;
        lastSoc               = -1.0;
        batteryCapacity       = 0.0;
        energyIdle            = 0.0;
        energySearching       = 0.0;
        energyAccompany       = 0.0;
        energyTalk            = 0.0;
        energyCharging        = 0.0;
        energyReturning       = 0.0;
        wasDriving            = false;
    }


    void onEvent(const int time, const des::EventType type, const std::string& /*message*/, const bool isDriving, const bool /*isCharging*/, const std::string& /*color*/ = "") override {
        if(isDriving && !wasDriving) {
            wasDriving = true;
            lastTimeMoved = time;
        } else if(!isDriving && wasDriving){
           moveTime += time - lastTimeMoved;
            wasDriving = false;
        }

        if(type == des::EventType::SIMULATION_END){
            publishReport();
        }
    };

    void onStateChanged(const int time, const des::RobotStateType& newState, des::BatteryProps batStats) override {
        const int passedTime = time - lastTimeStateChanged;

        if (lastSoc < 0.0) {
            batteryCapacity = batStats.capacity;
        }
        const double consumedAh = (lastSoc < 0.0) ? 0.0 : (lastSoc - batStats.soc) * batteryCapacity;

        switch (lastState) {
            case des::RobotStateType::CHARGING:
                chargingTime += passedTime;
                energyCharging += -consumedAh;
                break;
            case des::RobotStateType::ACCOMPANY:
                accompanyTime += passedTime;
                energyAccompany += consumedAh;
                break;
            case des::RobotStateType::SEARCHING:
                searchTime += passedTime;
                energySearching += consumedAh;
                break;
            case des::RobotStateType::IDLE:
                idleTime += passedTime;
                energyIdle += consumedAh;
                break;
            case des::RobotStateType::CONVERSATE:
                talkTime += passedTime;
                energyTalk += consumedAh;
                break;
            case des::RobotStateType::RETURNING:
                returningTime += passedTime;
                energyReturning += consumedAh;
                break;
        }
        lastSoc = batStats.soc;
        lastState = newState;
        lastTimeStateChanged = time;
    }

    void onMissionComplete(int /*time*/, const des::MissionState& state, const int timeDiff) override {
        switch (state) {
            case des::MissionState::COMPLETED:
                if (timeDiff >= 0) {
                    nMissionCompletedLate++;
                    accMissionToLateTime += timeDiff;
                    if (!hasLateMission || timeDiff < minLateness) minLateness = timeDiff;
                    if (!hasLateMission || timeDiff > maxLateness) maxLateness = timeDiff;
                    hasLateMission = true;
                } else {
                    nMissionCompleted++;
                    accMissionToEarlyTime += timeDiff;
                }
                break;
            case des::MissionState::CANCELLED:
                nMissionCanceled++;
                break;
            case des::MissionState::FAILED:
                nMissionFailed++;
                break;
            case des::MissionState::REJECTED:
                nMissionRejected++;
                break;
            default:
                break;
        }
    }

    void onRobotMoved(int /*time*/, const std::string& /*location*/, double distance) override {
        movedDistance += distance;
    }

    void publishReport() {
        auto msg = event_system_msgs::msg::MetricsReport();

        msg.idle_time          = idleTime;
        msg.moving_time        = moveTime;
        msg.searching_time     = searchTime;
        msg.accompany_time     = accompanyTime;
        msg.charging_time      = chargingTime;
        msg.talk_time          = talkTime;
        msg.total_missions     = nMissionCompleted + nMissionCompletedLate + nMissionFailed + nMissionCanceled + nMissionRejected;
        msg.missions_on_time   = nMissionCompleted;
        msg.missions_late      = nMissionCompletedLate;
        msg.missions_failed    = nMissionFailed;
        msg.missions_cancelled = nMissionCanceled;
        msg.missions_rejected  = nMissionRejected;

        msg.avg_early_arrival = (nMissionCompleted > 0) ? (float)accMissionToEarlyTime / nMissionCompleted : 0.0f;
        msg.avg_lateness = (nMissionCompletedLate > 0) ? (float)accMissionToLateTime / nMissionCompletedLate : 0.0f;
        msg.min_lateness = hasLateMission ? minLateness : 0;
        msg.max_lateness = hasLateMission ? maxLateness : 0;

        msg.rejected_rate = (msg.total_missions > 0) ? static_cast<float>(nMissionRejected) / msg.total_missions : 0.0f;

        msg.energy_idle_ah       = static_cast<float>(energyIdle);
        msg.energy_searching_ah  = static_cast<float>(energySearching);
        msg.energy_accompany_ah  = static_cast<float>(energyAccompany);
        msg.energy_talk_ah       = static_cast<float>(energyTalk);
        msg.energy_charging_ah   = static_cast<float>(energyCharging);
        msg.energy_total_consumed_ah = static_cast<float>(energyIdle + energySearching + energyAccompany + energyTalk + energyReturning);

        msg.total_distance = static_cast<float>(movedDistance);

        const int totalDriveTime = accompanyTime + searchTime + moveTime + returningTime;
        const auto totalTime = static_cast<double>(chargingTime + idleTime + totalDriveTime);
        if (totalTime > 0) {
            msg.utilization = static_cast<float>((searchTime + accompanyTime) / totalTime * 100.0);
        } else {
            msg.utilization = 0.0f;
        }


        RCLCPP_INFO(rclcpp::get_logger("Metrics"), "Publish Metrics");
        m_publisher->publish(msg);
        clear();
    }
};
