#pragma once

#include <iomanip>
#include <sstream>
#include <string>

namespace des {

enum class RobotStateType {
    IDLE,
    MOVING,
    ACCOMPANY,
    SEARCHING,
    CHARGING,
    CONVERSATE
};

struct Appointment {
    int id;
    std::string personName;
    std::string roomName;
    int appointmentTime;
    std::string description;
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
