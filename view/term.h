#pragma once

#include <iostream>
#include <string>
#include <iostream>
#include <iomanip>

#include "../observer/observer.h"

class TerminalView : public IObserver {
private:
    const std::string RESET   = "\033[0m";
    const std::string RED     = "\033[31m";
    const std::string GREEN   = "\033[32m";
    const std::string YELLOW  = "\033[33m";
    const std::string CYAN    = "\033[36m";

    std::string toHumanReadable(double totalSeconds) {
        int hours = static_cast<int>(totalSeconds / 3600.0);
        int minutes = static_cast<int>((totalSeconds - hours * 3600.0) / 60.0);
        int seconds = static_cast<int>(totalSeconds) % 60;

        std::ostringstream oss;
        oss << std::setw(2) << std::setfill('0') << hours << ":"
            << std::setw(2) << std::setfill('0') << minutes << ":"
            << std::setw(2) << std::setfill('0') << seconds;
        return oss.str();
    }

public:
    void onLog(int time, const std::string& message) override {
        std::cout << RED;
        std::cout << "[" << toHumanReadable(time) << "] " << message << std::endl;
        std::cout << RESET;
    }
};
