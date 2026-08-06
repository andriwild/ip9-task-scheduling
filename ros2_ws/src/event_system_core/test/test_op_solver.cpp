#include <gtest/gtest.h>

#include "op_fixtures.h"

#include "../src/algo/background/op_solver.h"

namespace {

OpInstance instanceWithDock() {
    std::vector<OpNode> nodes = {
        OpNode{"start", 0.0f, 0.0f, 0.0f},
        OpNode{"end", 0.0f, 0.0f, 0.0f},
        OpNode{"A", 1.0f, 0.0f, 0.0f},
        OpNode{"B", 1.0f, 0.0f, 0.0f},
        OpNode{"dock", 0.0f, 0.0f, 0.0f},
    };
    Mat mat = op_fixtures::lineMatrix({ 0.0f, 40.0f, 10.0f, 20.0f, 5.0f });
    const OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = 100000.0f,
        .energyBudget = 100000.0f,
        .initialSoc   = 100000.0f,
    };
    return OpInstance(std::move(nodes), std::move(mat), { 4 }, params);
}

OpInstance unreachableEnd(const std::vector<int>& stations) {
    std::vector<OpNode> nodes = {
        OpNode{"start", 0.0f, 0.0f, 0.0f},
        OpNode{"end", 0.0f, 0.0f, 0.0f},
        OpNode{"A", 1.0f, 0.0f, 0.0f},
        OpNode{"dock", 0.0f, 0.0f, 0.0f},
    };
    Mat mat = op_fixtures::lineMatrix({ 0.0f, 100.0f, 10.0f, 30.0f });
    const OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = 100000.0f,
        .energyBudget = 100000.0f,
        .initialSoc   = 30.0f,
        .maxEnergy    = 200.0f,
        .cvEnergy     = 200.0f,
        .driveSpeed   = 1.0f,
        .driveEnergy  = 1.0f,
    };
    return OpInstance(std::move(nodes), std::move(mat), stations, params);
}

std::vector<op_solver::detail::Cand> scoredValues(const std::vector<float>& values) {
    std::vector<op_solver::detail::Cand> scored;
    for (std::size_t i = 0; i < values.size(); ++i) {
        scored.push_back(op_solver::detail::Cand{ values[i], i, -1 });
    }
    return scored;
}

}  // namespace

TEST(TwoOpt, UncrossesTheRouteAndShortensTheDrive) {
    const OpInstance op = op_fixtures::lineInstance(90.0f);
    std::vector<int> tour = { 3, 2, 4 };
    EXPECT_FLOAT_EQ(op.routeDriveDistance(tour), 60.0f);

    op_solver::twoOpt(op, tour, 0, static_cast<int>(tour.size()));

    EXPECT_EQ(tour, (std::vector<int>{2, 3, 4}));
    EXPECT_FLOAT_EQ(op.routeDriveDistance(tour), 40.0f);
}

TEST(TwoOpt, OptimisesTheLegAfterAStationWithoutMovingIt) {
    const OpInstance op = instanceWithDock();
    std::vector<int> tour = { 4, 3, 2 };
    EXPECT_FLOAT_EQ(op.routeDriveDistance(tour), 60.0f);

    op_solver::twoOpt(op, tour, 1, static_cast<int>(tour.size()));

    EXPECT_EQ(tour, (std::vector<int>{4, 2, 3}));
    EXPECT_FLOAT_EQ(op.routeDriveDistance(tour), 40.0f);
}

TEST(TwoOpt, LeavesAnOptimalTourUntouched) {
    const OpInstance op = op_fixtures::lineInstance(90.0f);
    std::vector<int> tour = { 2, 3, 4 };

    op_solver::twoOpt(op, tour, 0, static_cast<int>(tour.size()));

    EXPECT_EQ(tour, (std::vector<int>{2, 3, 4}));
}

TEST(RestrictedCandidateList, AlphaZeroKeepsOnlyTheBest) {
    const auto rcl = op_solver::detail::restrictedCandidateList(scoredValues({ 5.0f, 10.0f, 15.0f }), 0.0f);
    ASSERT_EQ(rcl.size(), 1u);
    EXPECT_FLOAT_EQ(rcl.front().value, 15.0f);
}

TEST(RestrictedCandidateList, AlphaOneKeepsEverything) {
    EXPECT_EQ(op_solver::detail::restrictedCandidateList(scoredValues({ 5.0f, 10.0f, 15.0f }), 1.0f).size(), 3u);
}

TEST(RestrictedCandidateList, AlphaHalfKeepsTheUpperHalfOfTheValueRange) {
    const auto rcl = op_solver::detail::restrictedCandidateList(scoredValues({ 5.0f, 10.0f, 15.0f }), 0.5f);
    ASSERT_EQ(rcl.size(), 2u);
    EXPECT_FLOAT_EQ(rcl.front().value, 10.0f);
    EXPECT_FLOAT_EQ(rcl.back().value, 15.0f);
}

TEST(RepairToEnd, InsertsTheNearestStationWhenTheEndIsOutOfReach) {
    const OpInstance op = unreachableEnd({ 3 });
    std::vector<int> route = { 2 };
    ASSERT_FALSE(op.simulateRoute(route, true).feasible);

    op_solver::detail::repairToEnd(op, route);

    EXPECT_EQ(route, (std::vector<int>{2, 3}));
    EXPECT_TRUE(op.simulateRoute(route, true).feasible);
}

TEST(RepairToEnd, ShrinksTheRouteWhenThereIsNoStation) {
    const OpInstance op = unreachableEnd({});
    std::vector<int> route = { 2 };

    op_solver::detail::repairToEnd(op, route);

    EXPECT_TRUE(route.empty());
}

TEST(Grasp, SameSeedYieldsTheSameRoute) {
    const OpInstance op = op_fixtures::lineInstance(90.0f);

    EXPECT_EQ(op_solver::grasp(op, 5, 0.5f, 42), op_solver::grasp(op, 5, 0.5f, 42));
}

TEST(Grasp, TheReturnedRouteReachesTheEnd) {
    const OpInstance op = op_fixtures::lineInstance(90.0f);
    const std::vector<int> route = op_solver::grasp(op, 5, 0.5f, 42);

    ASSERT_FALSE(route.empty());
    EXPECT_TRUE(op.simulateRoute(route, true).feasible);
}

TEST(NearestStation, PicksTheClosestOfSeveral) {
    const OpInstance op = instanceWithDock();

    EXPECT_EQ(op_solver::nearestStation(op, 2), 4);
}
