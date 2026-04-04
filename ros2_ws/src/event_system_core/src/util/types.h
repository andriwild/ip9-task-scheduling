#pragma once

#include <iomanip>
#include <vector>
#include <string>

#include "../model/person.h"

namespace des {

struct Point {
    double m_x, m_y, m_yaw;
    Point() = default;

    Point(const double pnt, const double pnt1, const double yaw) : m_x(pnt), m_y(pnt1), m_yaw(yaw) {}

    friend std::ostream& operator<<(std::ostream& os, const Point& s) {
        os << "(" << s.m_x << ", " << s.m_y << ", " << s.m_yaw << ")";
        return os;
    }
};

struct Segment {
    int id;
    Point m_points[2];

    Segment() = default;

    Segment(const int segment_id, const Point& p1, const Point& p2) : id(segment_id), m_points{p1, p2} {}

    friend std::ostream& operator<<(std::ostream& os, const Segment& s) {
        os << "Segment id=" << s.id << ", p1=" << s.m_points[0] << ", p2=" << s.m_points[1];
        return os;
    }
};

struct Location {
    std::string m_name;
    Point m_p;

    explicit Location(const std::string& name, const Point p) : m_name(name), m_p(p) {}

    friend std::ostream& operator<<(std::ostream& os, const Location& l) {
        os << l.m_name << l.m_p;
        return os;
    }
};

enum class DistributionType {
    NORMAL,
    UNIFORM,
    EXPONENTIAL,
    LOG_NORMAL
};

inline std::string distributionTypeToString(const DistributionType type) {
    switch (type) {
        case DistributionType::UNIFORM:     return "uniform";
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

struct SimConfig {
    double robotSpeed;
    double robotAccompanySpeed;
    double driveTimeStd;
    double conversationProbability;
    double conversationDurationStd;
    double conversationDurationMean;
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
    double appointmentDuration = 1800;

    friend std::ostream& operator<<(std::ostream& os, const SimConfig& config) {
        const int W = 30;
        os << "\n"
           << "\033[1m" << "--- Configuration Loaded ---" << "\033[0m" << std::endl;
        os << std::left << std::setw(W) << "robotSpeed" << ": " << config.robotSpeed << std::endl;
        os << std::left << std::setw(W) << "robotAccompanySpeed" << ": " << config.robotAccompanySpeed << std::endl;
        os << std::left << std::setw(W) << "driveTimeStd" << ": " << config.driveTimeStd << std::endl;
        os << std::left << std::setw(W) << "conversationProbability" << ": " << config.conversationProbability << std::endl;
        os << std::left << std::setw(W) << "conversationDurationStd" << ": " << config.conversationDurationStd << std::endl;
        os << std::left << std::setw(W) << "conversationDurationMean" << ": " << config.conversationDurationMean << std::endl;
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
        os << std::left << std::setw(W) << "dockPose" << ": " << config.dockLocation<< std::endl;
        os << std::left << std::setw(W) << "cache enabled" << ": " << config.cacheEnabled << std::endl;
        os << std::left << std::setw(W) << "appointmentsPath" << ": " << config.appointmentsPath << std::endl;
        os << std::left << std::setw(W) << "appointmentDuration" << ": " << config.appointmentDuration << std::endl;
        os << "----------------------------\n"
           << std::endl;
        return os;
    }
};

enum class RobotStateType {
    IDLE,
    ACCOMPANY,
    SEARCHING,
    CHARGING,
    CONVERSATE
};

enum class RobotSubState {
    STANDING,
    DRIVING
};

enum class Result {
    FAILURE,
    RUNNING,
    SUCCESS
};

enum class EventType : int {
    SIMULATION_START = 0,
    SIMULATION_END = 1,
    STOP_DRIVE = 2,
    MISSION_COMPLETE = 3,
    MISSION_DISPATCH = 4,
    DROP_OFF_CONV_COMPLETE = 5,
    FOUND_PERSON_CONV_COMPLETE = 6,
    ABORT_SEARCH = 7,
    START_DROP_OFF_CONV = 8,
    START_FOUND_PERSON_CONV = 9,
    START_ACCOMPANY = 10,
    ARRIVED_ACCOMPANY = 11,
    START_DRIVE = 12,
    BATTERY_FULL = 13,
    RESET = 14,
    PERSON_TRANSITION = 15,
    PERSON_ARRIVED = 16,
    PERSON_DEPARTURE = 17,
    MISSION_START = 18,
    APPOINTMENT_END = 19
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

struct Appointment {
    int id;
    std::string personName;
    std::string roomName;
    int appointmentTime;
    std::string description;
    MissionState state = PENDING;

    friend std::ostream& operator<<(std::ostream& os, const Appointment& appt) {
        constexpr int W = 25;
        os << "\n" << "\033[1m" << "--- Appointment ---" << "\033[0m" << std::endl;
        os << std::left << std::setw(W) << "id" << ": " << appt.id << std::endl;
        os << std::left << std::setw(W) << "state" << ": " << appt.state << std::endl;
        os << std::left << std::setw(W) << "description" << ": " << appt.description << std::endl;
        os << std::left << std::setw(W) << "time" << ": " << appt.appointmentTime << std::endl;
        os << std::left << std::setw(W) << "personName" << ": " << appt.personName << std::endl;
        os << std::left << std::setw(W) << "roomName" << ": " << appt.roomName << std::endl;
        return os;
    }
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
