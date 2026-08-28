#pragma once

#include <cassert>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

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
    RANDOM_SECTOR,
    TRUE_DISTRIBUTION,
    UNIFORM
};

inline std::string searchRewardStrategyToString(const SearchRewardStrategy strategy) {
    switch (strategy) {
        case SearchRewardStrategy::FREQUENCY:         return "frequency";
        case SearchRewardStrategy::RANDOM:            return "random";
        case SearchRewardStrategy::RANDOM_SECTOR:     return "random_sector";
        case SearchRewardStrategy::TRUE_DISTRIBUTION: return "true_distribution";
        case SearchRewardStrategy::UNIFORM:           return "uniform";
        default:                                      return "beta_smoothed";
    }
}

inline SearchRewardStrategy searchRewardStrategyFromString(const std::string& str) {
    if (str == "frequency") return SearchRewardStrategy::FREQUENCY;
    if (str == "random") return SearchRewardStrategy::RANDOM;
    if (str == "random_sector") return SearchRewardStrategy::RANDOM_SECTOR;
    if (str == "true_distribution") return SearchRewardStrategy::TRUE_DISTRIBUTION;
    if (str == "uniform") return SearchRewardStrategy::UNIFORM;
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

enum class ConversationKind {
    FOUND_PERSON,
    DROP_OFF,
    ASK_DIRECTIONS
};

enum class ChargeTrigger {
    OPPORTUNISTIC,
    PLANNED,
    REACTIVE
};

inline std::string chargeTriggerToString(const ChargeTrigger trigger) {
    switch (trigger) {
        case ChargeTrigger::OPPORTUNISTIC:  return "Opportunistic";
        case ChargeTrigger::PLANNED:        return "Planned";
        case ChargeTrigger::REACTIVE:       return "Reactive";
    }
    assert(false);
}

struct ChargeSession {
    int time;
    ChargeTrigger trigger;
    double soc;
};

inline std::optional<ConversationKind> conversationKindFromString(const std::string& value) {
    if (value == "found_person") {
        return ConversationKind::FOUND_PERSON;
    }
    if (value == "drop_off") {
        return ConversationKind::DROP_OFF;
    }
    if (value == "ask_directions") {
        return ConversationKind::ASK_DIRECTIONS;
    }
    return std::nullopt;
}

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
    SCAN                       = 36,
    CONVERSATION_START         = 37,
    CONVERSATION_END           = 38
};

enum OrderState {
    PENDING,
    COMPLETED,
    IN_PROGRESS,
    FAILED,
    CANCELLED,
    REJECTED
};

inline std::string orderStateStr(const OrderState state) {
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
