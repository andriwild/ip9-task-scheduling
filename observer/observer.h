#pragma once

#include <string>

class IObserver {
public:
    virtual ~IObserver() = default;
    virtual void onLog(int time, const std::string& message) = 0;
    virtual void onRobotMoved(int time, std::string location) = 0;
};
