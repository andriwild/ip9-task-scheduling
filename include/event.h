#pragma once

struct Point {
    double x, y;
};


struct SearchEvent {
    int eventId;
    Point location;
};

struct event {
    int eventId;
    int time;
    Point location;
    int personId;
    std::vector<std::shared_ptr<SearchEvent>> children;
};