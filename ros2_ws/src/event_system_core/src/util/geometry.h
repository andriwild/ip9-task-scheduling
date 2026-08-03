#pragma once

#include <optional>
#include <random>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>

#include "types.h"
#include "rnd.h"

namespace geom {

namespace bg = boost::geometry;
using BgPoint = bg::model::d2::point_xy<double>;
using BgPolygon = bg::model::polygon<BgPoint>;

inline BgPolygon toPolygon(const des::Polygon& poly) {
    BgPolygon p;
    for (const auto& v : poly) {
        bg::append(p.outer(), BgPoint(v.m_x, v.m_y));
    }
    bg::correct(p);
    return p;
}

inline bool isVisible(const des::Point& p, const des::Polygon& visibility) {
    if (visibility.size() < 3) {
        return true;
    }
    return bg::covered_by(BgPoint(p.m_x, p.m_y), toPolygon(visibility));
}

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
