/*
 * Drive destination strategy used by DriveEvent.
 * RoomTarget resolves a room name through the path planner,
 * PointTarget drives straight to a coordinate.
 * travel() estimates, arrive() applies the effects of having arrived.
 *
 */

#pragma once

#include <cmath>
#include <string>
#include <utility>

#include "engine/contracts/i_sim_context.h"
#include "model/person.h"
#include "model/robot.h"
#include "plugins/order_registry.h"
#include "util/types.h"

namespace des {

class DriveTarget {
public:
    virtual ~DriveTarget() = default;

    virtual std::pair<int, double> travel(ISimContext& ctx) const = 0;
    virtual void onStart(ISimContext& /*ctx*/) const {}
    virtual void arrive(ISimContext& ctx, double distance) const = 0;
    virtual std::string label() const = 0;
};

inline void observeSightings(ISimContext& ctx) {
    for (const auto& [name, personRoom] : ctx.getAllPersonLocations()) {
        const Person* person = ctx.getPersonByName(name);
        if (person && person->busy) {
            continue;
        }
        ctx.getRobot()->observePerson(ctx.getTime(), name, ctx.robotSeesPerson(name));
    }
}

class RoomTarget final : public DriveTarget {
    std::string m_room;

public:
    explicit RoomTarget(std::string room) : m_room(std::move(room)) {}

    std::pair<int, double> travel(ISimContext& ctx) const override {
        const Journey j = ctx.scheduleArrival(m_room);
        return { static_cast<int>(j.duration), j.distance };
    }

    void onStart(ISimContext& ctx) const override {
        ctx.getRobot()->setTargetLocation(m_room);
        if (const auto order = ctx.getOrderPtr()) {
            OrderRegistry::instance().get(order->type).onStartDriveEvent(ctx, *order);
        }
    }

    void arrive(ISimContext& ctx, double distance) const override {
        ctx.robotMoved(m_room, distance);
        ctx.getRobot()->setVisibility(ctx.room(m_room).m_tour.visibilityAt(0));
        ctx.setBTBlackboard("location", m_room);
        if (const auto order = ctx.getOrderPtr()) {
            OrderRegistry::instance().get(order->type).onStopDriveEvent(ctx, *order);
        }
        ctx.getRobot()->beginRoomVisit(m_room);
        observeSightings(ctx);
    }

    [[nodiscard]] std::string label() const override { return m_room; }
};

class PointTarget final : public DriveTarget {
    Point m_point;
    Polygon m_visibility;
    double m_speed;

public:
    explicit PointTarget(const Point& point, Polygon visibility, const double speed = 0.0)
        : m_point(point), m_visibility(std::move(visibility)), m_speed(speed) {}

    std::pair<int, double> travel(ISimContext& ctx) const override {
        const Point from = ctx.getRobot()->getPosition();
        const double dist = std::hypot(m_point.m_x - from.m_x, m_point.m_y - from.m_y);
        const double speed = m_speed > 0.0 ? m_speed : ctx.getRobot()->getCurrentSpeed();
        const int duration = std::max(1, static_cast<int>(std::lround(dist / speed)));
        return { duration, dist };
    }

    void arrive(ISimContext& ctx, double distance) const override {
        ctx.robotMovedTo(m_point, distance);
        ctx.getRobot()->setVisibility(m_visibility);
        observeSightings(ctx);
    }

    [[nodiscard]] std::string label() const override {
        return "(" + std::to_string(m_point.m_x) + ", " + std::to_string(m_point.m_y) + ")";
    }
};

}  // namespace des
