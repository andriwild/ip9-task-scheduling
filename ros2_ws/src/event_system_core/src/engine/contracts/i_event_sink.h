#pragma once

#include <memory>

class IEvent;

class IEventSink {
public:
    virtual ~IEventSink() = default;
    virtual void pushEvent(const std::shared_ptr<IEvent>& event) = 0;

    // Push an event that marks the end of the robot's current activity
    // (StopDrive, BatteryFull, End*Event). Tracked as the in-flight commitment
    // so that interrupts can shift it. For everything else, use pushEvent.
    virtual void startActivity(const std::shared_ptr<IEvent>& endEvent) = 0;
};
