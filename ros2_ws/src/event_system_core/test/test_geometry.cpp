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

TEST(GeometryApproachPoint, StopsAtTheGivenDistanceFromTheTarget) {
    const des::Point stop = geom::approachPoint(des::Point{0.0, 0.0, 0.0}, des::Point{5.0, 0.0, 0.0}, 1.0);
    EXPECT_DOUBLE_EQ(stop.m_x, 1.0);
    EXPECT_DOUBLE_EQ(stop.m_y, 0.0);
}

TEST(GeometryApproachPoint, StaysOnTheLineBetweenBothPoints) {
    const des::Point stop = geom::approachPoint(des::Point{0.0, 0.0, 0.0}, des::Point{6.0, 8.0, 0.0}, 5.0);
    EXPECT_DOUBLE_EQ(stop.m_x, 3.0);
    EXPECT_DOUBLE_EQ(stop.m_y, 4.0);
    EXPECT_DOUBLE_EQ(std::hypot(stop.m_x, stop.m_y), 5.0);
}

TEST(GeometryApproachPoint, ZeroDistanceKeepsTheTarget) {
    const des::Point stop = geom::approachPoint(des::Point{2.0, 3.0, 0.0}, des::Point{9.0, 3.0, 0.0}, 0.0);
    EXPECT_DOUBLE_EQ(stop.m_x, 2.0);
    EXPECT_DOUBLE_EQ(stop.m_y, 3.0);
}

TEST(GeometryApproachPoint, AlreadyCloserThanTheDistanceMeansNoMove) {
    const des::Point stop = geom::approachPoint(des::Point{0.0, 0.0, 0.0}, des::Point{0.5, 0.0, 0.0}, 1.0);
    EXPECT_DOUBLE_EQ(stop.m_x, 0.5);
    EXPECT_DOUBLE_EQ(stop.m_y, 0.0);
}
