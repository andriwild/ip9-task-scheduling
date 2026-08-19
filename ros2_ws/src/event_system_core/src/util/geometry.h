#pragma once

#include <cmath>
#include <optional>
#include <random>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>

#include "types.h"
#include "rnd.h"
#include "util/point.h"

namespace geom {

namespace bg = boost::geometry;
using BgPoint = bg::model::d2::point_xy<double>;
using BgPolygon = bg::model::polygon<BgPoint>;

// convert to boost polygon
inline BgPolygon toPolygon(const des::Polygon& poly) {
    BgPolygon p;
    for (const auto& v : poly) {
        bg::append(p.outer(), BgPoint(v.m_x, v.m_y));
    }
    bg::correct(p);
    return p;
}

// check if a point is in a polygon
inline bool isVisible(const des::Point& p, const des::Polygon& visibility) {
    if (visibility.size() < 3) {
        return true;
    }
    return bg::covered_by(BgPoint(p.m_x, p.m_y), toPolygon(visibility));
}

// point on the line from target towards from, at the given distance from target
inline des::Point approachPoint(const des::Point& target, const des::Point& from, const double distance) {
    const double dx = from.m_x - target.m_x;
    const double dy = from.m_y - target.m_y;
    const double gap = std::hypot(dx, dy);
    if (distance <= 0.0 || gap <= 0.0) {
        return target;
    }
    if (gap <= distance) {
        return from;
    }
    const double t = distance / gap;
    return des::Point{ target.m_x + dx * t, target.m_y + dy * t, from.m_yaw };
}

// generate a randomly distributed point in a given polygon
inline std::optional<des::Point> sampleInPolygon(const des::Polygon& poly, std::mt19937& rng, const int maxTries = 30) {
    if (poly.size() < 3) {
        return std::nullopt;
    }
    const BgPolygon p = toPolygon(poly);
    bg::model::box<BgPoint> box;
    bg::envelope(p, box);
    const double minX = box.min_corner().x(), minY = box.min_corner().y();
    const double maxX = box.max_corner().x(), maxY = box.max_corner().y();
    for (int i = 0; i < maxTries; ++i) {
        const BgPoint pt(rnd::uni(rng, minX, maxX), rnd::uni(rng, minY, maxY));
        if (bg::within(pt, p)) {
            return des::Point{ pt.x(), pt.y(), 0.0 };
        }
    }
    return std::nullopt;
}

}  // namespace geom
