#pragma once

#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "../model/person.h"
#include "../model/room.h"
#include "point.h"

namespace des {

using Mat = std::vector<std::vector<float>>;

enum class DistributionType {
    NORMAL,
    UNIFORM,
    EXPONENTIAL,
    LOG_NORMAL
};

inline std::string distributionTypeToString(const DistributionType type) {
    switch (type) {
        case DistributionType::UNIFORM:      return "uniform";
        case DistributionType::NORMAL:       return "normal";
        case DistributionType::EXPONENTIAL:  return "exponential";
        case DistributionType::LOG_NORMAL:   return "log_normal";
        default: return "normal";
    }
}

inline DistributionType distributionTypeFromString(const std::string& str) {
    if (str == "uniform") return DistributionType::UNIFORM;
    if (str == "exponential") return DistributionType::EXPONENTIAL;
    if (str == "log_normal") return DistributionType::LOG_NORMAL;
    return DistributionType::NORMAL;
}

enum class SearchRewardStrategy {
    BETA_SMOOTHED,
    FREQUENCY,
    RANDOM,
    RANDOM_SECTOR
};

inline std::string searchRewardStrategyToString(const SearchRewardStrategy strategy) {
    switch (strategy) {
        case SearchRewardStrategy::FREQUENCY:     return "frequency";
        case SearchRewardStrategy::RANDOM:        return "random";
        case SearchRewardStrategy::RANDOM_SECTOR: return "random_sector";
        default:                                  return "beta_smoothed";
    }
}

inline SearchRewardStrategy searchRewardStrategyFromString(const std::string& str) {
    if (str == "frequency") return SearchRewardStrategy::FREQUENCY;
    if (str == "random") return SearchRewardStrategy::RANDOM;
    if (str == "random_sector") return SearchRewardStrategy::RANDOM_SECTOR;
    return SearchRewardStrategy::BETA_SMOOTHED;
}

enum class SearchRouteStrategy {
    COST_AWARE,
    PROBABILITY_ONLY
};

inline std::string searchRouteStrategyToString(const SearchRouteStrategy strategy) {
    switch (strategy) {
        case SearchRouteStrategy::PROBABILITY_ONLY: return "probability_only";
        default:                                    return "cost_aware";
    }
}

inline SearchRouteStrategy searchRouteStrategyFromString(const std::string& str) {
    if (str == "probability_only") return SearchRouteStrategy::PROBABILITY_ONLY;
    return SearchRouteStrategy::COST_AWARE;
}

enum class EnergyReserveStrategy {
    NEXT_MISSION,
    HORIZON
};

inline std::string energyReserveStrategyToString(const EnergyReserveStrategy strategy) {
    switch (strategy) {
        case EnergyReserveStrategy::NEXT_MISSION: return "next_mission";
        default:                                  return "horizon";
    }
}

inline EnergyReserveStrategy energyReserveStrategyFromString(const std::string& str) {
    if (str == "next_mission") return EnergyReserveStrategy::NEXT_MISSION;
    return EnergyReserveStrategy::HORIZON;
}

constexpr unsigned int ROBOT_SEED_OFFSET     = 1;
constexpr unsigned int PLACEMENT_SEED_OFFSET = 2;
constexpr unsigned int GRASP_SEED_OFFSET     = 3;
constexpr unsigned int PERSON_SEED_BASE      = 100;
constexpr unsigned int ROUND_SEED_STRIDE     = 1000;

enum class RoundMode {
    REPLICATION,
    CONTINUATION
};

inline std::string roundModeToString(const RoundMode mode) {
    switch (mode) {
        case RoundMode::CONTINUATION: return "continuation";
        default:                      return "replication";
    }
}

inline RoundMode roundModeFromString(const std::string& str) {
    if (str == "continuation") return RoundMode::CONTINUATION;
    return RoundMode::REPLICATION;
}

enum class ExecutionMode {
    SCHEDULED,
    BACKGROUND,
    INTERRUPT
};

struct SimConfig {
    double robotSpeed;
    double driveTimeStd;
    double timeBuffer;
    double energyConsumptionDrive;
    double energyConsumptionBase;
    double batteryCapacity;
    double initialBatteryCapacity;
    double chargingRate;
    double lowBatteryThreshold;
    double fullBatteryThreshold;
    double arrivalMean;
    double arrivalStd;
    double departureMean;
    double departureStd;
    DistributionType arrivalDistribution;
    DistributionType departureDistribution;
    std::string dockLocation;
    bool cacheEnabled;
    std::string appointmentsPath;
    std::string peopleSpawnLocation;
    double personIdentificationRange = 5.0;
    double personRecognitionRange = 5.0;
    double personSpeed = 1.4;
    int simStartTime = 25200;  // 07:00
    int simDuration  = 43200;
    bool useDistanceMatrix = false;
    double batteryVoltage = 12.0;
    double cvThreshold = 0.8;
    double taperFraction = 0.5;
    bool chargeToFull = true;
    bool alwaysChargeAtDock = false;
    bool metricsCsvExport = true;
    bool debugExport = false;
    bool replanBackgroundOnInterrupt = true;
    double lunchMean = 12.0 * 3600;
    double lunchStd = 1800.0;
    DistributionType lunchDistribution = DistributionType::NORMAL;
    double lunchDurationMean = 2400.0;
    double lunchDurationStd = 600.0;
    std::vector<std::string> searchExcludedRooms = {"Elevator", "Stairwell", "Dock"};
    std::string employeesPath = "";
    SearchRewardStrategy searchRewardStrategy = SearchRewardStrategy::BETA_SMOOTHED;
    SearchRouteStrategy searchRouteStrategy = SearchRouteStrategy::COST_AWARE;
    bool searchRolePrior = false;
    double searchPriorWeight = 4.0;
    double searchWorkplacePrior = 0.6;
    double personDirectionsProbability = 0.0;
    EnergyReserveStrategy energyReserveStrategy = EnergyReserveStrategy::HORIZON;
    int energyReserveHorizon = 4 * 3600;
    unsigned int seed = 42;
    int rounds = 1;
    RoundMode roundMode = RoundMode::REPLICATION;

    friend std::ostream& operator<<(std::ostream& os, const SimConfig& config) {
        const int W = 30;
        os << "\n"
           << "\033[1m" << "--- Configuration Loaded ---" << "\033[0m" << std::endl;
        os << std::left << std::setw(W) << "robotSpeed" << ": " << config.robotSpeed << std::endl;
        os << std::left << std::setw(W) << "driveTimeStd" << ": " << config.driveTimeStd << std::endl;
        os << std::left << std::setw(W) << "timeBuffer" << ": " << config.timeBuffer << std::endl;
        os << std::left << std::setw(W) << "energyConsumptionDrive" << ": " << config.energyConsumptionDrive << std::endl;
        os << std::left << std::setw(W) << "energyConsumptionBase" << ": " << config.energyConsumptionBase << std::endl;
        os << std::left << std::setw(W) << "batteryCapacity" << ": " << config.batteryCapacity << std::endl;
        os << std::left << std::setw(W) << "initialBatteryCapacity" << ": " << config.initialBatteryCapacity << std::endl;
        os << std::left << std::setw(W) << "chargingRate" << ": " << config.chargingRate << std::endl;
        os << std::left << std::setw(W) << "lowBatteryThreshold" << ": " << config.lowBatteryThreshold << std::endl;
        os << std::left << std::setw(W) << "fullBatteryThreshold" << ": " << config.fullBatteryThreshold << std::endl;
        os << std::left << std::setw(W) << "arrivalMean" << ": " << config.arrivalMean << std::endl;
        os << std::left << std::setw(W) << "arrivalStd" << ": " << config.arrivalStd << std::endl;
        os << std::left << std::setw(W) << "departureMean" << ": " << config.departureMean << std::endl;
        os << std::left << std::setw(W) << "departureStd" << ": " << config.departureStd << std::endl;
        os << std::left << std::setw(W) << "arrivalDistribution" << ": " << distributionTypeToString(config.arrivalDistribution) << std::endl;
        os << std::left << std::setw(W) << "departureDistribution" << ": " << distributionTypeToString(config.departureDistribution) << std::endl;
        os << std::left << std::setw(W) << "lunchMean" << ": " << config.lunchMean << std::endl;
        os << std::left << std::setw(W) << "lunchStd" << ": " << config.lunchStd << std::endl;
        os << std::left << std::setw(W) << "lunchDistribution" << ": " << distributionTypeToString(config.lunchDistribution) << std::endl;
        os << std::left << std::setw(W) << "lunchDurationMean" << ": " << config.lunchDurationMean << std::endl;
        os << std::left << std::setw(W) << "lunchDurationStd" << ": " << config.lunchDurationStd << std::endl;
        os << std::left << std::setw(W) << "dockPose" << ": " << config.dockLocation<< std::endl;
        os << std::left << std::setw(W) << "cache enabled" << ": " << config.cacheEnabled << std::endl;
        os << std::left << std::setw(W) << "appointmentsPath" << ": " << config.appointmentsPath << std::endl;
        os << std::left << std::setw(W) << "employeesPath" << ": " << config.employeesPath << std::endl;
        os << std::left << std::setw(W) << "searchRewardStrategy" << ": " << searchRewardStrategyToString(config.searchRewardStrategy) << std::endl;
        os << std::left << std::setw(W) << "searchRouteStrategy" << ": " << searchRouteStrategyToString(config.searchRouteStrategy) << std::endl;
        os << std::left << std::setw(W) << "searchRolePrior" << ": " << config.searchRolePrior << std::endl;
        os << std::left << std::setw(W) << "energyReserveStrategy" << ": " << energyReserveStrategyToString(config.energyReserveStrategy) << std::endl;
        os << std::left << std::setw(W) << "energyReserveHorizon" << ": " << config.energyReserveHorizon << std::endl;
        os << std::left << std::setw(W) << "seed" << ": " << config.seed << std::endl;
        os << std::left << std::setw(W) << "rounds" << ": " << config.rounds << std::endl;
        os << std::left << std::setw(W) << "roundMode" << ": " << roundModeToString(config.roundMode) << std::endl;
        os << std::left << std::setw(W) << "peopleSpawnLocation" << ": " << config.peopleSpawnLocation << std::endl;
        os << std::left << std::setw(W) << "personIdentificationRange" << ": " << config.personIdentificationRange << std::endl;
        os << std::left << std::setw(W) << "personRecognitionRange" << ": " << config.personRecognitionRange << std::endl;
        os << std::left << std::setw(W) << "personDirectionsProbability" << ": " << config.personDirectionsProbability << std::endl;
        os << std::left << std::setw(W) << "personSpeed" << ": " << config.personSpeed << std::endl;
        os << std::left << std::setw(W) << "simStartTime" << ": " << config.simStartTime << std::endl;
        os << std::left << std::setw(W) << "simDuration" << ": " << config.simDuration << std::endl;
        os << std::left << std::setw(W) << "useDistanceMatrix" << ": " << config.useDistanceMatrix << std::endl;
        os << std::left << std::setw(W) << "batteryVoltage" << ": " << config.batteryVoltage << std::endl;
        os << std::left << std::setw(W) << "cvThreshold" << ": " << config.cvThreshold << std::endl;
        os << std::left << std::setw(W) << "taperFraction" << ": " << config.taperFraction << std::endl;
        os << std::left << std::setw(W) << "chargeToFull" << ": " << config.chargeToFull << std::endl;
        os << std::left << std::setw(W) << "alwaysChargeAtDock" << ": " << config.alwaysChargeAtDock << std::endl;
        os << std::left << std::setw(W) << "metricsCsvExport" << ": " << config.metricsCsvExport << std::endl;
        os << std::left << std::setw(W) << "replanBackgroundOnInterrupt" << ": " << config.replanBackgroundOnInterrupt << std::endl;
        os << std::left << std::setw(W) << "searchExcludedRooms" << ": ";
        for (size_t i = 0; i < config.searchExcludedRooms.size(); ++i) {
            os << (i ? ", " : "") << config.searchExcludedRooms[i];
        }
        os << std::endl;
        os << "----------------------------\n"
           << std::endl;
        return os;
    }
};

// Structural categories of robot states — used for BT control flow
enum class RobotStateType {
    IDLE,
    MISSION,
    CHARGING
};

enum class Result {
    FAILURE,
    RUNNING,
    SUCCESS
};

enum class EventType : int {
    SIMULATION_START           = 0, 
    SIMULATION_END             = 1, 
    STOP_DRIVE                 = 2, 
    MISSION_COMPLETE           = 3, 
    MISSION_DISPATCH           = 4, 
    DROP_OFF_CONV_COMPLETE     = 5, 
    FOUND_PERSON_CONV_COMPLETE = 6, 
    ABORT_SEARCH               = 7, 
    START_DROP_OFF_CONV        = 8, 
    START_FOUND_PERSON_CONV    = 9, 
    START_ACCOMPANY            = 10,
    ARRIVED_ACCOMPANY          = 11,
    START_DRIVE                = 12,
    BATTERY_FULL               = 13,
    RESET                      = 14,
    PERSON_TRANSITION          = 15,
    PERSON_ARRIVED             = 16,
    PERSON_DEPARTURE           = 17,
    MISSION_START              = 18,
    APPOINTMENT_END            = 19,
    PERSON_ACCOMPANY_DEPARTURE = 22,
    PERSON_ACCOMPANY_ARRIVED   = 23,
    DATA_ACQUISITION_START     = 24,
    DATA_ACQUISITION           = 25,
    CLEAN_START                = 26,
    CLEAN                      = 27,
    ORDER_ARRIVAL              = 28,
    INFORMATION_START          = 29,
    INFORMATION                = 30,
    CHARGE_MISSION_START       = 31,
    CHARGE_MISSION             = 32,
    CHARGE_PHASE_TRANSITION    = 33,
    PERSON_ROOM_ARRIVED        = 34,
    BACKGROUND_RELEASE         = 35,
    SCAN                       = 36
};

enum MissionState {
    PENDING,
    COMPLETED,
    IN_PROGRESS,
    FAILED,
    CANCELLED,
    REJECTED
};

inline std::string missionStateStr(const MissionState state) {
    switch(state) {
        case PENDING: return "Pending";
        case COMPLETED: return "Completed";
        case IN_PROGRESS: return "In Progress";
        case FAILED: return "Failed";
        case CANCELLED: return "Cancelled";
        case REJECTED: return "Rejected";
        default: return "Unknown";
    };
};

struct BatteryProps {
    double soc;
    double capacity;
    double lowThreshold;
};

using PersonList  = std::vector<std::unique_ptr<Person>>;
using PersonMap   = std::map<std::string, Person*>;

inline std::string toHumanReadableTime(const int sec, const bool includeSeconds = true) {
    const int hours   = static_cast<int>(sec / 3600.0);
    const int minutes = static_cast<int>((sec - hours * 3600.0) / 60.0);
    const int seconds = static_cast<int>(sec) % 60;

    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << hours << ":" << std::setw(2) << std::setfill('0')
        << minutes;
    if (includeSeconds) {
        oss << ":" << std::setw(2) << std::setfill('0') << seconds;
    }
    return oss.str();
}
}  // namespace des
