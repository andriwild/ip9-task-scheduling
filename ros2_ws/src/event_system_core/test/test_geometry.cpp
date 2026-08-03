#include <gtest/gtest.h>

#include "../src/util/geometry.h"

namespace {

des::Polygon square(const double x0, const double y0, const double x1, const double y1) {
    return {
        des::Point{x0, y0, 0.0},
        des::Point{x1, y0, 0.0},
        des::Point{x1, y1, 0.0},
        des::Point{x0, y1, 0.0}
    };
}

}  // namespace

TEST(GeometryVisibility, EmptyPolygonMeansUnobstructedSight) {
    EXPECT_TRUE(geom::isVisible(des::Point{100.0, 100.0, 0.0}, des::Polygon{}));
}

TEST(GeometryVisibility, DegeneratePolygonMeansUnobstructedSight) {
    const des::Polygon line = { des::Point{0.0, 0.0, 0.0}, des::Point{1.0, 0.0, 0.0} };
    EXPECT_TRUE(geom::isVisible(des::Point{100.0, 100.0, 0.0}, line));
}

TEST(GeometryVisibility, PointInsideIsVisible) {
    EXPECT_TRUE(geom::isVisible(des::Point{2.0, 2.0, 0.0}, square(0.0, 0.0, 4.0, 4.0)));
}

TEST(GeometryVisibility, PointOutsideIsHidden) {
    EXPECT_FALSE(geom::isVisible(des::Point{5.0, 2.0, 0.0}, square(0.0, 0.0, 4.0, 4.0)));
}

TEST(GeometryVisibility, PointOnTheBorderIsVisible) {
    EXPECT_TRUE(geom::isVisible(des::Point{4.0, 2.0, 0.0}, square(0.0, 0.0, 4.0, 4.0)));
}
