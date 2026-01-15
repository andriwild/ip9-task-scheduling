#pragma once

#include "../observer/observer.h"
#include <iostream>

class Metrics:  public IObserver {
    int searchTime = 0;
    int accompanyTime = 0;
    int idleTime = 0;
    int moveTime = 0;
    int talkTime = 0;
    int chargingTime = 0;

    RobotStateType lastState;
    int lastTimeStateChanged = 0;

    int accMissionToLateTime = 0;
    int accMissionToEarlyTime = 0;

    int nMissionCompleted = 0;
    int nMissionFailed = 0;
    int nMissionCompletedLate = 0;
    int nMissionCaceled = 0;

    double movedDistance = 0.0;

public:

    std::string getName() override {
        return "Metrics";
    }

    void show() {
        int totalDriveTime = accompanyTime + searchTime + moveTime;
        double totalTime = static_cast<double>(chargingTime +  idleTime + totalDriveTime);
        
        std::cout << "\n" << "\033[1;33m" << "=== SIMULATION METRICS REPORT ===" << "\033[0m" << "\n\n";

        std::cout << "\033[1m" << "--- Robot State Distribution ---" << "\033[0m" << "\n";
        std::cout 
            << std::left  << std::setw(15) << "State" 
            << std::right << std::setw(15) << "Duration" 
            << std::setw(10) << "%" << "\n";
        std::cout << "----------------------------------------\n";


        printRow(totalTime, "Idle",      idleTime);
        printRow(totalTime, "Moving",    moveTime);
        printRow(totalTime, "Searching", searchTime);
        printRow(totalTime, "Accompany", accompanyTime);
        printRow(totalTime, "Charging",  chargingTime);

        std::cout << "----------------------------------------\n";
        std::cout 
            << std::left  << std::setw(15) << "TOTAL TIME" 
            << std::right << std::setw(15) << fmtTime(totalTime) 
            << "\n\n";

        int totalMissions = nMissionCompleted + nMissionCompletedLate + nMissionFailed + nMissionCaceled; 
        double successRate = (totalMissions > 0) 
            ? ((double)(nMissionCompleted + nMissionCompletedLate) / totalMissions * 100.0) 
            : 0.0;

        double avgEarly = (nMissionCompleted > 0)     ? (double)accMissionToEarlyTime / nMissionCompleted : 0.0;
        double avgLate  = (nMissionCompletedLate > 0) ? (double)accMissionToLateTime  / nMissionCompletedLate : 0.0;

        std::cout << "\033[1m" << "--- Mission Performance ---" << "\033[0m" << "\n";
        std::cout << "Total Missions:   " << totalMissions << "\n";
        std::cout << "  - On Time:      " << "\033[32m" << nMissionCompleted << "\033[0m" << "\n";
        std::cout << "  - Late:         " << "\033[33m" << nMissionCompletedLate << "\033[0m" << "\n";
        std::cout << "  - Failed:       " << "\033[31m" << nMissionFailed << "\033[0m" << "\n";
        std::cout << "Success Rate:     " << std::fixed << std::setprecision(1) << successRate << "%\n";
        std::cout << "\n";

        std::cout << "Avg Early Arrival: " << std::fixed << std::setprecision(1) << avgEarly << "s\n";
        std::cout << "Avg Lateness:      " << std::fixed << std::setprecision(1) << avgLate << "s\n";

        std::cout << "\n";
        std::cout << "\033[1m" << "--- Movement ---" << "\033[0m" << "\n";
        std::cout << "Total Distance:    " << std::fixed << std::setprecision(2) << movedDistance << " meters\n";

        std::cout << "\n";
        std::cout << "\033[1m" << "--- General ---" << "\033[0m" << "\n";
        std::cout << "Utilization:       " << std::fixed << std::setprecision(2) << (searchTime + accompanyTime) / totalTime * 100 << " %\n";
        std::cout << "=================================\n\n";
    }


    void onStateChanged(int time, const RobotStateType& newState) override {
        int passedTime = time - lastTimeStateChanged;
        
        switch(lastState) {
            case RobotStateType::CHARGING:   chargingTime += passedTime; break;
            case RobotStateType::ACCOMPANY:  accompanyTime   += passedTime; break;
            case RobotStateType::SEARCHING:  searchTime   += passedTime; break;
            case RobotStateType::IDLE:       idleTime     += passedTime; break;
            case RobotStateType::MOVING:     moveTime     += passedTime; break;
            case RobotStateType::CONVERSATE: talkTime     += passedTime; break;
        }

        lastState = newState;
        lastTimeStateChanged = time;
    };

    void onMissionComplete(int time, des::MissionState& state, int timeDiff) override {
        switch(state){
            case des::MissionState::COMPLETED:
                if(timeDiff > 0) {
                    nMissionCompletedLate ++;
                    accMissionToLateTime += timeDiff;
                } else {
                    nMissionCompleted ++;
                    accMissionToEarlyTime += timeDiff;
                }
                break;
            case des::MissionState::CANCELLED: nMissionCaceled ++; break;
            case des::MissionState::FAILED:    nMissionFailed ++;  break;
            default: // do nothing
                break;
        }
    };


    void onRobotMoved(int time, const std::string& location, double distance) override {
        movedDistance += distance;
    };

private: 

    std::string fmtTime (int s) {
            int m = s / 60;
            int sec = s % 60;
            std::ostringstream oss;
            oss << m << "m " << std::setw(2) << std::setfill('0') << sec << "s";
            return oss.str();
        };


        void printRow (double totalTime, const std::string& label, int val) {
            double pct = (val / totalTime) * 100.0;
            std::cout 
                << std::left  << std::setw(15) << label 
                << std::right << std::setw(15) << fmtTime(val) 
                << std::setw(9) << std::fixed << std::setprecision(1) << pct << "%" << "\n";
        };
};
