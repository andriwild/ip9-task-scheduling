#pragma once

#include <iostream>
#include <string>
#include <iostream>

#include "../util/types.h"
#include "../observer/observer.h"

class TerminalView : public IObserver {
    const std::string RESET   = "\033[0m";
    const std::string RED     = "\033[31m";
    const std::string GREEN   = "\033[32m";
    const std::string YELLOW  = "\033[33m";
    const std::string CYAN    = "\033[36m";

public:
    void onLog(int time, const std::string& message) override {
        std::cout << RED;
        std::cout << "[" << des::toHumanReadableTime(time) << "] " << message << std::endl;
        std::cout << RESET;
    }

    void onRobotMoved(int time, const std::string& location, double distance) override {
        std::cout << GREEN;
        std::cout << "[" << des::toHumanReadableTime(time) << "] " << location << std::endl;
        std::cout << RESET;
    };

};
