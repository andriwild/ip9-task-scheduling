#pragma once

#include <cmath>
#include <string>
#include <utility>

#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/sighting.h"
#include "plugins/order_registry.h"
#include "util/types.h"

class DriveTarget {
public:
    virtual ~DriveTarget() = default;

    virtual std::pair<int, double> travel(ISimContext& ctx) const = 0;
    virtual void onStart(ISimContext& /*ctx*/) const {}
    virtual void arrive(ISimContext& ctx, double distance) const = 0;
    virtual std::string label() const = 0;
};

inline void recordSightings(ISimContext& ctx, const std::string& room) {
    for (const auto& [name, personRoom] : ctx.getAllPersonLocations()) {
        ctx.getRobot()->addSighting({
            ctx.getTime(),
            name,
            room,
            ctx.robotSeesPerson(name) ? SightingKind::PRESENT : SightingKind::ABSENT
        });
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
        recordSightings(ctx, m_room);
    }

    std::string label() const override { return m_room; }
};

class PointTarget final : public DriveTarget {
    des::Point m_point;
    des::Polygon m_visibility;

public:
    explicit PointTarget(const des::Point& point, des::Polygon visibility)
        : m_point(point), m_visibility(std::move(visibility)) {}

    std::pair<int, double> travel(ISimContext& ctx) const override {
        const des::Point from = ctx.getRobot()->getPosition();
        const double dist = std::hypot(m_point.m_x - from.m_x, m_point.m_y - from.m_y);
        const double speed = ctx.getRobot()->getCurrentSpeed();
        const int duration = std::max(1, static_cast<int>(std::lround(dist / speed)));
        return { duration, dist };
    }

    void arrive(ISimContext& ctx, double distance) const override {
        ctx.robotMovedTo(m_point, distance);
        ctx.getRobot()->setVisibility(m_visibility);
        recordSightings(ctx, ctx.getRobot()->getLocation());
    }

    std::string label() const override {
        return "(" + std::to_string(m_point.m_x) + ", " + std::to_string(m_point.m_y) + ")";
    }
};
