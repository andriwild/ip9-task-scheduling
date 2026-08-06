#include <gtest/gtest.h>

#include "op_fixtures.h"

#include "../src/algo/search/search_solver.h"

namespace {

OpInstance taskFarFromTheEnd(const float distanceToEnd, const float timeBudget) {
    std::vector<OpNode> nodes = {
        OpNode{"start", 0.0f, 0.0f, 0.0f},
        OpNode{"end", 0.0f, 0.0f, 0.0f},
        OpNode{"A", 1.0f, 0.0f, 0.0f},
    };
    Mat mat = op_fixtures::uniformMatrix(nodes.size(), 10.0f);
    mat[2][1] = distanceToEnd;
    mat[1][2] = distanceToEnd;
    const OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = timeBudget,
        .energyBudget = 1000.0f,
        .initialSoc   = 1000.0f,
    };
    return OpInstance(std::move(nodes), std::move(mat), {}, params);
}

OpInstance reserveBoundTasks() {
    std::vector<OpNode> nodes = {
        OpNode{"start", 0.0f, 0.0f, 0.0f},
        OpNode{"end", 0.0f, 0.0f, 0.0f},
        OpNode{"cheap", 1.0f, 0.0f, 10.0f},
        OpNode{"costly", 1.0f, 0.0f, 80.0f},
    };
    Mat mat = op_fixtures::uniformMatrix(nodes.size(), 10.0f);
    const OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = 100000.0f,
        .energyBudget = 100000.0f,
        .initialSoc   = 100.0f,
        .endSocMin    = 50.0f,
    };
    return OpInstance(std::move(nodes), std::move(mat), {}, params);
}

}  // namespace

TEST(GreedySearchOrder, VisitsTheCheapTaskFirstEvenWhenItComesLast) {
    const std::vector<int> route = op_solver::greedySearchOrder(op_fixtures::twoTasks(500.0f, 10.0f));
    ASSERT_EQ(route.size(), 2u);
    EXPECT_EQ(route.front(), 3);
}

TEST(GreedySearchOrder, TaskIsSkippedWhenTheDriveToTheEndExceedsTheBudget) {
    EXPECT_EQ(op_solver::greedySearchOrder(taskFarFromTheEnd(10.0f, 100.0f)).size(), 1u);
    EXPECT_TRUE(op_solver::greedySearchOrder(taskFarFromTheEnd(500.0f, 100.0f)).empty());
}

TEST(GreedySearchOrder, TaskBeyondTheEnergyReserveIsSkipped) {
    const std::vector<int> route = op_solver::greedySearchOrder(reserveBoundTasks());
    ASSERT_EQ(route.size(), 1u);
    EXPECT_EQ(route.front(), 2);
}

TEST(GreedySearchOrder, FollowsTheBestValueChain) {
    EXPECT_EQ(op_solver::greedySearchOrder(op_fixtures::lineInstance(90.0f)), (std::vector<int>{3, 2}));
}

TEST(GreedySearchOrder, StopsWhenTheTimeBudgetIsExhausted) {
    EXPECT_EQ(op_solver::greedySearchOrder(op_fixtures::lineInstance(55.0f)), (std::vector<int>{3}));
}
