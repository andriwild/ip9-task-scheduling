// Generated with Claude Code (Anthropic), then reviewed and adapted by the author. See the index of auxiliary tools.
#include <gtest/gtest.h>

#include "op_fixtures.h"

#include "../src/algo/op_solver_common.h"

namespace {

des::op::OpInstance instanceWithDock() {
    std::vector<des::op::OpNode> nodes = {
        des::op::OpNode{"start", 0.0f, 0.0f, 0.0f},
        des::op::OpNode{"end", 0.0f, 0.0f, 0.0f},
        des::op::OpNode{"task", 1.0f, 0.0f, 0.0f},
        des::op::OpNode{"dock", 0.0f, 0.0f, 0.0f},
    };
    des::Mat mat = op_fixtures::uniformMatrix(nodes.size(), 10.0f);
    const des::op::OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = 1000.0f,
        .energyBudget = 1000.0f,
        .initialSoc   = 1000.0f,
    };
    return des::op::OpInstance(std::move(nodes), std::move(mat), { 3 }, params);
}

}  // namespace

TEST(GreedyValue, CheapServiceBeatsExpensiveServiceAtEqualDistance) {
    const des::op::OpInstance op = op_fixtures::twoTasks(10.0f, 500.0f);
    EXPECT_GT(des::op::greedyValue(op, 0, 2), des::op::greedyValue(op, 0, 3));
}

TEST(GreedyValue, TimeGovernsWhenEnergyIsAmple) {
    const des::op::OpInstance op = op_fixtures::slowVersusHungry(1000.0f, 1000000.0f);
    EXPECT_GT(des::op::greedyValue(op, 0, 3), des::op::greedyValue(op, 0, 2));
}

TEST(GreedyValue, EnergyGovernsWhenTheRemainingChargeIsTight) {
    const des::op::OpInstance op = op_fixtures::slowVersusHungry(1000000.0f, 200.0f);
    EXPECT_GT(des::op::greedyValue(op, 0, 2), des::op::greedyValue(op, 0, 3));
}

TEST(GreedyValue, IdenticalNodesAtIdenticalDistanceTie) {
    const des::op::OpInstance op = op_fixtures::twoTasks(10.0f, 10.0f);
    EXPECT_FLOAT_EQ(des::op::greedyValue(op, 0, 2), des::op::greedyValue(op, 0, 3));
}

TEST(TaskCandidates, AnchorsAndDocksAreExcluded) {
    EXPECT_EQ(des::op::taskCandidates(instanceWithDock()), (std::vector<int>{2}));
}

TEST(TaskCandidates, EveryTaskNodeIsListed) {
    EXPECT_EQ(des::op::taskCandidates(op_fixtures::lineInstance(90.0f)), (std::vector<int>{2, 3, 4}));
}
