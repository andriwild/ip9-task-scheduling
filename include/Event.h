#pragma once

struct Point {
    double x, y;
};

struct Event {
    int eventId;
    double time;
    Point location;

    Event(const int eventId, const double time, const Point location):
    eventId(eventId),
    time(time),
    location(location)
    {}

    virtual ~Event() = default;
};

struct EscortEvent final : Event {
    int personId;
    EscortEvent(const int eventId, const double time, const Point location, const int personId):
    Event(eventId, time, location),
    personId(personId) {}
};


struct SearchEvent final : Event {
    int personId;
    SearchEvent(const int eventId, const double time, const Point location, const int personId ):
    Event(eventId, time, location), personId(personId) {}
};


struct DriveStart final : Event {
    DriveStart(const int eventId, const double time, const Point &location)
        : Event(eventId, time, location) {
    }
};

struct DriveEnd final : Event {
    DriveEnd(const int eventId, const double time, const Point &location)
        : Event(eventId, time, location) {
    }
};