#pragma once


class SimTime final {
public:
    SimTime() : t(0) {}
    void inc() {
        t++;
    }
    int getTime() const {
        return t;
    }

private:
    int t;
};

class ReadOnlyClock final {
    SimTime* clock;

public:
    explicit ReadOnlyClock(SimTime* clock):
    clock(clock)
    {}

    int getTime() const {
        return clock->getTime();
    }
};