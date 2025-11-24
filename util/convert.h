#include <string>
#include <iostream>
#include <iomanip>

inline std::string toHumanReadable(double totalSeconds) {
    int hours = static_cast<int>(totalSeconds / 3600.0);
    int minutes = static_cast<int>((totalSeconds - hours * 3600.0) / 60.0);
    int seconds = static_cast<int>(totalSeconds) % 60;

    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << hours << ":"
        << std::setw(2) << std::setfill('0') << minutes << ":"
        << std::setw(2) << std::setfill('0') << seconds;
    return oss.str();
}
