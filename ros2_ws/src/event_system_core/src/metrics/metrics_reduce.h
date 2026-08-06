#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "engine/contracts/i_event.h"
#include "model/state_log.h"
#include "util/types.h"

namespace des::reduce {

struct StateTotals {
    std::map<std::string, int>    timeByName;
    std::map<std::string, double> energyByName;
    std::map<des::RobotStateType, int>    timeByCategory;
    std::map<des::RobotStateType, double> energyByCategory;
    int totalTime = 0;
};

struct BatteryTotals {
    int    cyclesTotal    = 0;
    int    cyclesComplete = 0;
    int    deepDischarge  = 0;
    double dischargedAh   = 0.0;
    double avgDepthOfDischarge = 0.0;
    double equivalentFullCycles = 0.0;
};

struct MissionStats {
    int registered = 0;
    int onTime     = 0;
    int late       = 0;
    int failed     = 0;
    int cancelled  = 0;
    int rejected   = 0;
    std::map<std::string, int> failByDetail;
};

struct MissionTotals {
    std::map<des::ExecutionMode, MissionStats> byMode;
    std::map<int, std::map<des::ExecutionMode, MissionStats>> byDay;
    int accLateTime  = 0;
    int accEarlyTime = 0;
    int minLateness  = 0;
    int maxLateness  = 0;
    bool hasLate     = false;
    int chargeCyclesComplete = 0;
};

struct DriveTotals {
    int movingTime  = 0;
    double distance = 0.0;
    std::map<int, double> distanceByDay;
};

struct DaySpan {
    double energyAh   = 0.0;
    double minSoc     = -1.0;
    int    idleTime    = 0;
    int    missionTime = 0;
    int    chargingTime = 0;
    int    totalTime   = 0;
};

MissionTotals missions(const EventList& protocol);
DriveTotals   drives(const EventList& protocol);
std::map<int, int> chargeCyclesByDay(const std::vector<int>& chargeSessions);
StateTotals   states(const StateLog& log, double capacity);
BatteryTotals battery(const StateLog& log, const std::vector<int>& chargeSessions, double capacity, double lowThreshold, double dischargedAh);
std::map<int, DaySpan> perDay(const StateLog& log, double capacity);

}  // namespace des::reduce
