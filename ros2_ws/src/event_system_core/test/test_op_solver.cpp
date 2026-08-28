// Generated with Claude Code (Anthropic), then reviewed and adapted by the author. See the index of auxiliary tools.
#include <gtest/gtest.h>

#include "op_fixtures.h"

#include "../src/algo/background/op_solver.h"

namespace {

des::op::OpInstance instanceWithDock() {
    std::vector<des::op::OpNode> nodes = {
        des::op::OpNode{"start", 0.0f, 0.0f, 0.0f},
        des::op::OpNode{"end", 0.0f, 0.0f, 0.0f},
        des::op::OpNode{"A", 1.0f, 0.0f, 0.0f},
        des::op::OpNode{"B", 1.0f, 0.0f, 0.0f},
        des::op::OpNode{"dock", 0.0f, 0.0f, 0.0f},
    };
    des::Mat mat = op_fixtures::lineMatrix({ 0.0f, 40.0f, 10.0f, 20.0f, 5.0f });
    const des::op::OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = 100000.0f,
        .energyBudget = 100000.0f,
        .initialSoc   = 100000.0f,
    };
    return des::op::OpInstance(std::move(nodes), std::move(mat), { 4 }, params);
}

des::op::OpInstance unreachableEnd(const std::vector<int>& docks) {
    std::vector<des::op::OpNode> nodes = {
        des::op::OpNode{"start", 0.0f, 0.0f, 0.0f},
        des::op::OpNode{"end", 0.0f, 0.0f, 0.0f},
        des::op::OpNode{"A", 1.0f, 0.0f, 0.0f},
        des::op::OpNode{"dock", 0.0f, 0.0f, 0.0f},
    };
    des::Mat mat = op_fixtures::lineMatrix({ 0.0f, 100.0f, 10.0f, 30.0f });
    const des::op::OpParams params {
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
    return des::op::OpInstance(std::move(nodes), std::move(mat), docks, params);
}

std::vector<des::op::Cand> scoredValues(const std::vector<float>& values) {
    std::vector<des::op::Cand> scored;
    for (std::size_t i = 0; i < values.size(); ++i) {
        scored.push_back(des::op::Cand{ values[i], i, -1 });
    }
    return scored;
}

}  // namespace

TEST(TwoOpt, UncrossesTheRouteAndShortensTheDrive) {
    const des::op::OpInstance op = op_fixtures::lineInstance(90.0f);
    std::vector<int> tour = { 3, 2, 4 };
    EXPECT_FLOAT_EQ(op.routeDriveDistance(tour), 60.0f);

    des::op::twoOpt(op, tour, 0, static_cast<int>(tour.size()));

    EXPECT_EQ(tour, (std::vector<int>{2, 3, 4}));
    EXPECT_FLOAT_EQ(op.routeDriveDistance(tour), 40.0f);
}

TEST(TwoOpt, OptimisesTheLegAfterADockWithoutMovingIt) {
    const des::op::OpInstance op = instanceWithDock();
    std::vector<int> tour = { 4, 3, 2 };
    EXPECT_FLOAT_EQ(op.routeDriveDistance(tour), 60.0f);

    des::op::twoOpt(op, tour, 1, static_cast<int>(tour.size()));

    EXPECT_EQ(tour, (std::vector<int>{4, 2, 3}));
    EXPECT_FLOAT_EQ(op.routeDriveDistance(tour), 40.0f);
}

TEST(TwoOpt, LeavesAnOptimalTourUntouched) {
    const des::op::OpInstance op = op_fixtures::lineInstance(90.0f);
    std::vector<int> tour = { 2, 3, 4 };

    des::op::twoOpt(op, tour, 0, static_cast<int>(tour.size()));

    EXPECT_EQ(tour, (std::vector<int>{2, 3, 4}));
}

TEST(RestrictedCandidateList, AlphaZeroKeepsOnlyTheBest) {
    const auto rcl = des::op::restrictedCandidateList(scoredValues({ 5.0f, 10.0f, 15.0f }), 0.0f);
    ASSERT_EQ(rcl.size(), 1u);
    EXPECT_FLOAT_EQ(rcl.front().value, 15.0f);
}

TEST(RestrictedCandidateList, AlphaOneKeepsEverything) {
    EXPECT_EQ(des::op::restrictedCandidateList(scoredValues({ 5.0f, 10.0f, 15.0f }), 1.0f).size(), 3u);
}

TEST(RestrictedCandidateList, AlphaHalfKeepsTheUpperHalfOfTheValueRange) {
    const auto rcl = des::op::restrictedCandidateList(scoredValues({ 5.0f, 10.0f, 15.0f }), 0.5f);
    ASSERT_EQ(rcl.size(), 2u);
    EXPECT_FLOAT_EQ(rcl.front().value, 10.0f);
    EXPECT_FLOAT_EQ(rcl.back().value, 15.0f);
}

TEST(RepairToEnd, InsertsTheNearestDockWhenTheEndIsOutOfReach) {
    const des::op::OpInstance op = unreachableEnd({ 3 });
    std::vector<int> route = { 2 };
    ASSERT_FALSE(op.simulateRoute(route, true).feasible);

    des::op::repairToEnd(op, route);

    EXPECT_EQ(route, (std::vector<int>{2, 3}));
    EXPECT_TRUE(op.simulateRoute(route, true).feasible);
}

TEST(RepairToEnd, ShrinksTheRouteWhenThereIsNoDock) {
    const des::op::OpInstance op = unreachableEnd({});
    std::vector<int> route = { 2 };

    des::op::repairToEnd(op, route);

    EXPECT_TRUE(route.empty());
}

TEST(Grasp, SameSeedYieldsTheSameRoute) {
    const des::op::OpInstance op = op_fixtures::lineInstance(90.0f);

    EXPECT_EQ(des::op::grasp(op, 5, 0.5f, 42), des::op::grasp(op, 5, 0.5f, 42));
}

TEST(Grasp, TheReturnedRouteReachesTheEnd) {
    const des::op::OpInstance op = op_fixtures::lineInstance(90.0f);
    const std::vector<int> route = des::op::grasp(op, 5, 0.5f, 42);

    ASSERT_FALSE(route.empty());
    EXPECT_TRUE(op.simulateRoute(route, true).feasible);
}

TEST(NearestDock, PicksTheClosestOfSeveral) {
    const des::op::OpInstance op = instanceWithDock();

    EXPECT_EQ(des::op::nearestDock(op, 2), 4);
}
