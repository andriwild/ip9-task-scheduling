// Generated with Claude Code (Anthropic), then reviewed and adapted by the author. See the index of auxiliary tools.
#include <gtest/gtest.h>

#include "op_fixtures.h"

#include "../src/algo/search/search_solver.h"

namespace {

des::op::OpInstance taskFarFromTheEnd(const float distanceToEnd, const float timeBudget) {
    std::vector<des::op::OpNode> nodes = {
        des::op::OpNode{"start", 0.0f, 0.0f, 0.0f},
        des::op::OpNode{"end", 0.0f, 0.0f, 0.0f},
        des::op::OpNode{"A", 1.0f, 0.0f, 0.0f},
    };
    des::Mat mat = op_fixtures::uniformMatrix(nodes.size(), 10.0f);
    mat[2][1] = distanceToEnd;
    mat[1][2] = distanceToEnd;
    const des::op::OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = timeBudget,
        .energyBudget = 1000.0f,
        .initialSoc   = 1000.0f,
    };
    return des::op::OpInstance(std::move(nodes), std::move(mat), {}, params);
}

des::op::OpInstance reserveBoundTasks() {
    std::vector<des::op::OpNode> nodes = {
        des::op::OpNode{"start", 0.0f, 0.0f, 0.0f},
        des::op::OpNode{"end", 0.0f, 0.0f, 0.0f},
        des::op::OpNode{"cheap", 1.0f, 0.0f, 10.0f},
        des::op::OpNode{"costly", 1.0f, 0.0f, 80.0f},
    };
    des::Mat mat = op_fixtures::uniformMatrix(nodes.size(), 10.0f);
    const des::op::OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = 100000.0f,
        .energyBudget = 100000.0f,
        .initialSoc   = 100.0f,
        .endSocMin    = 50.0f,
    };
    return des::op::OpInstance(std::move(nodes), std::move(mat), {}, params);
}

}  // namespace

TEST(GreedySearchOrder, VisitsTheCheapTaskFirstEvenWhenItComesLast) {
    const std::vector<int> route = des::op::greedySearchOrder(op_fixtures::twoTasks(500.0f, 10.0f));
    ASSERT_EQ(route.size(), 2u);
    EXPECT_EQ(route.front(), 3);
}

TEST(GreedySearchOrder, TaskIsSkippedWhenTheDriveToTheEndExceedsTheBudget) {
    EXPECT_EQ(des::op::greedySearchOrder(taskFarFromTheEnd(10.0f, 100.0f)).size(), 1u);
    EXPECT_TRUE(des::op::greedySearchOrder(taskFarFromTheEnd(500.0f, 100.0f)).empty());
}

TEST(GreedySearchOrder, TaskBeyondTheEnergyReserveIsSkipped) {
    const std::vector<int> route = des::op::greedySearchOrder(reserveBoundTasks());
    ASSERT_EQ(route.size(), 1u);
    EXPECT_EQ(route.front(), 2);
}

TEST(GreedySearchOrder, FollowsTheBestValueChain) {
    EXPECT_EQ(des::op::greedySearchOrder(op_fixtures::lineInstance(90.0f)), (std::vector<int>{3, 2}));
}

TEST(GreedySearchOrder, StopsWhenTheTimeBudgetIsExhausted) {
    EXPECT_EQ(des::op::greedySearchOrder(op_fixtures::lineInstance(55.0f)), (std::vector<int>{3}));
}
