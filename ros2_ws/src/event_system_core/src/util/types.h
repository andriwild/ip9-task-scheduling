#pragma once

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

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

struct SimConfig {
    double personFindProbability;
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
    std::vector<double> dockPose;
    bool cacheEnabled;
    std::string appointmentsPath;

    friend std::ostream& operator<<(std::ostream& os, const SimConfig& config) {
        const int W = 30;
        os << "\n"
           << "\033[1m" << "--- Configuration Loaded ---" << "\033[0m" << std::endl;
        os << std::left << std::setw(W) << "personFindProbability" << ": " << config.personFindProbability << std::endl;
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
        os << std::left << std::setw(W) << "dockPose" << ": ["
           << (config.dockPose.size() > 0 ? config.dockPose[0] : 0) << ", "
           << (config.dockPose.size() > 1 ? config.dockPose[1] : 0) << ", "
           << (config.dockPose.size() > 2 ? config.dockPose[2] : 0) << "]" << std::endl;
        os << std::left << std::setw(W) << "cache enabled" << ": " << config.cacheEnabled << std::endl;
        os << std::left << std::setw(W) << "appointmentsPath" << ": " << config.appointmentsPath << std::endl;
        os << "----------------------------\n"
           << std::endl;
        return os;
    }
};

enum class RobotStateType {
    IDLE,
    MOVING,
    ACCOMPANY,
    SEARCHING,
    CHARGING,
    CONVERSATE
};

enum class EventType : int {
    SIMULATION_START = 0,
    SIMULATION_END = 1,
    ARRIVED = 2,
    MISSION_COMPLETE = 3,
    MISSION_DISPATCH = 4,
    DROP_OFF_CONV_COMPLETE = 5,
    FOUND_PERSON_CONV_COMPLETE = 6,
    ABORT_SEARCH = 7,
    START_DROP_OFF_CONV = 8,
    START_FOUND_PERSON_CONV = 9,
    START_ACCOMPANY = 10
};

struct Person {
    int id;
    std::string firstName;
    std::string lastName;
    std::string birthDate;
    std::string sex;
    int assignedRoomId;
};

enum MissionState {
    PENDING,
    COMPLETED,
    IN_PROGRESS,
    FAILED,
    CANCELLED
};

struct Appointment {
    int id;
    std::string personName;
    std::string roomName;
    int appointmentTime;
    std::string description;
    MissionState state = PENDING;

    friend std::ostream& operator<<(std::ostream& os, const Appointment& appt) {
        const int W = 25;
        os << "\n"
           << "\033[1m" << "--- Appointment ---" << "\033[0m" << std::endl;
        os << std::left << std::setw(W) << "id" << ": " << appt.id << std::endl;
        os << std::left << std::setw(W) << "state" << ": " << appt.state << std::endl;
        os << std::left << std::setw(W) << "description" << ": " << appt.description << std::endl;
        os << std::left << std::setw(W) << "time" << ": " << appt.appointmentTime << std::endl;
        os << std::left << std::setw(W) << "personName" << ": " << appt.personName << std::endl;
        os << std::left << std::setw(W) << "roomName" << ": " << appt.roomName << std::endl;
        return os;
    }
};

inline std::string toHumanReadableTime(const int sec, bool includeSeconds = true) {
    int hours = static_cast<int>(sec / 3600.0);
    int minutes = static_cast<int>((sec - hours * 3600.0) / 60.0);
    int seconds = static_cast<int>(sec) % 60;

    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << hours << ":" << std::setw(2) << std::setfill('0')
        << minutes;
    if (includeSeconds) {
        oss << ":" << std::setw(2) << std::setfill('0') << seconds;
    }
    return oss.str();
}
}  // namespace des
