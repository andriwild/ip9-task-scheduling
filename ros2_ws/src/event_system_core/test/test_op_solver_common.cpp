#include <gtest/gtest.h>

#include "op_fixtures.h"

#include "../src/algo/op_solver_common.h"

namespace {

des::OpInstance instanceWithStation() {
    std::vector<des::OpNode> nodes = {
        des::OpNode{"start", 0.0f, 0.0f, 0.0f},
        des::OpNode{"end", 0.0f, 0.0f, 0.0f},
        des::OpNode{"task", 1.0f, 0.0f, 0.0f},
        des::OpNode{"dock", 0.0f, 0.0f, 0.0f},
    };
    des::Mat mat = op_fixtures::uniformMatrix(nodes.size(), 10.0f);
    const des::OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = 1000.0f,
        .energyBudget = 1000.0f,
        .initialSoc   = 1000.0f,
    };
    return des::OpInstance(std::move(nodes), std::move(mat), { 3 }, params);
}

}  // namespace

TEST(GreedyValue, CheapServiceBeatsExpensiveServiceAtEqualDistance) {
    const des::OpInstance op = op_fixtures::twoTasks(10.0f, 500.0f);
    EXPECT_GT(des::op_solver::greedyValue(op, 0, 2), des::op_solver::greedyValue(op, 0, 3));
}

TEST(GreedyValue, TimeGovernsWhenEnergyIsAmple) {
    const des::OpInstance op = op_fixtures::slowVersusHungry(1000.0f, 1000000.0f);
    EXPECT_GT(des::op_solver::greedyValue(op, 0, 3), des::op_solver::greedyValue(op, 0, 2));
}

TEST(GreedyValue, EnergyGovernsWhenTheRemainingChargeIsTight) {
    const des::OpInstance op = op_fixtures::slowVersusHungry(1000000.0f, 200.0f);
    EXPECT_GT(des::op_solver::greedyValue(op, 0, 2), des::op_solver::greedyValue(op, 0, 3));
}

TEST(GreedyValue, IdenticalNodesAtIdenticalDistanceTie) {
    const des::OpInstance op = op_fixtures::twoTasks(10.0f, 10.0f);
    EXPECT_FLOAT_EQ(des::op_solver::greedyValue(op, 0, 2), des::op_solver::greedyValue(op, 0, 3));
}

TEST(TaskCandidates, AnchorsAndStationsAreExcluded) {
    EXPECT_EQ(des::op_solver::detail::taskCandidates(instanceWithStation()), (std::vector<int>{2}));
}

TEST(TaskCandidates, EveryTaskNodeIsListed) {
    EXPECT_EQ(des::op_solver::detail::taskCandidates(op_fixtures::lineInstance(90.0f)), (std::vector<int>{2, 3, 4}));
}
