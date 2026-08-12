#pragma once

#include <algorithm>
#include <format>
#include <iterator>
#include <optional>
#include <string>

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "util/rnd.h"
#include "util/constants.h"

namespace des {

inline int busyRetryAt(ISimContext& ctx, const int now) {
    return now + static_cast<int>(rnd::uni(ctx.robotRng(), 60, 300));
}

inline double personWalkTime(ISimContext& ctx, Person& person, const std::string& from, const std::string& to) {
    const std::optional<double> dist = ctx.getDistance(from, to);
    const auto config = ctx.getConfig();
    const double speed = person.sex == "female" ? config->personSpeedFemale : config->personSpeedMale;
    if (!dist.has_value() || speed <= 0.0) {
        return 0.0;
    }
    const double t = dist.value() / speed;
    const double tRnd = rnd::normal(person.rng, t, t * 0.1);
    return tRnd < 0 ? t : tRnd;
}

inline std::optional<std::string> drawNextRoom(Person& person, const std::string& currentRoom) {
    const auto it = std::ranges::find(person.roomLabels, currentRoom);
    if (it == person.roomLabels.end()) {
        return std::nullopt;
    }
    const auto currentIndex = std::distance(person.roomLabels.begin(), it);
    const std::vector<double>& row = person.transitionMatrix.at(currentIndex);
    return person.roomLabels.at(rnd::discrete_dist(person.rng, row));
}

class PersonEvent : public IEvent {
public:
    Person* const person;
    std::string targetRoom;

    // Set on every move, read by the move trace.
    // OUTDOOR and IN_TRANSIT are no rooms, so they have a name but no position.
    std::string m_room;
    std::optional<Point> m_at;

    explicit PersonEvent(const int time, Person* p) :
        IEvent(time),
        person(p)
    {}

    void moveTo(ISimContext& ctx, const std::string& room) {
        ctx.setPersonLocation(person->firstName, room);
        m_room = room;
        m_at = ctx.getPersonPosition(person->firstName);
    }

    void retryLater(ISimContext& ctx) const {
        ctx.pushEvent(withTime(busyRetryAt(ctx, time)));
    }

    void scheduleTransition(ISimContext& ctx, int at) const;

    [[nodiscard]] std::string getName() const override {
        return std::format("{} walking to {}", person->firstName, targetRoom);
    }
    [[nodiscard]] std::string getColor() const override { return person->color; }
    [[nodiscard]] int getMissionId() const override { return -1; }
};

}  // namespace des
