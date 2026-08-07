#include <gtest/gtest.h>

#include "op_fixtures.h"

namespace {

des::OpInstance chargeInstance() {
    std::vector<des::OpNode> nodes = {
        des::OpNode{"start", 0.0f, 0.0f, 0.0f},
        des::OpNode{"end", 0.0f, 0.0f, 0.0f},
        des::OpNode{"dock", 0.0f, 0.0f, 0.0f},
    };
    des::Mat mat = op_fixtures::uniformMatrix(nodes.size(), 10.0f);
    const des::OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = 1000.0f,
        .energyBudget = 1000.0f,
        .initialSoc   = 100.0f,
        .maxEnergy    = 100.0f,
        .chargeTimePerWh = 1.0f,
        .chargeTimePerWhTapered = 2.0f,
        .cvEnergy     = 80.0f,
        .driveSpeed   = 1.0f,
        .driveEnergy  = 1.0f,
    };
    return des::OpInstance(std::move(nodes), std::move(mat), { 2 }, params);
}

}  // namespace

TEST(OpInstanceRoute, CostsMatchTheHandComputedValues) {
    const des::OpInstance op = op_fixtures::lineInstance(90.0f);
    const std::vector<int> route = { 3, 2 };
    const des::OpInstance::Sim sim = op.simulateRoute(route, true);

    EXPECT_TRUE(sim.feasible);
    EXPECT_FLOAT_EQ(sim.time, 80.0f);
    EXPECT_FLOAT_EQ(op.routeDriveDistance(route), 60.0f);
    EXPECT_FLOAT_EQ(op.routeReward(route), 5.0f);
}

TEST(OpInstanceRoute, TheFullRouteExceedsTheTimeBudget) {
    const des::OpInstance op = op_fixtures::lineInstance(90.0f);
    const des::OpInstance::Sim sim = op.simulateRoute({ 3, 2, 4 }, true);

    EXPECT_FALSE(sim.feasible);
    EXPECT_FLOAT_EQ(sim.time, 100.0f);
}

TEST(OpInstanceRoute, DriveToTheEndIsOnlyCountedWithToEnd) {
    const des::OpInstance op = op_fixtures::lineInstance(90.0f);

    EXPECT_FLOAT_EQ(op.simulateRoute({ 3, 2 }, false).time, 50.0f);
    EXPECT_FLOAT_EQ(op.simulateRoute({ 3, 2 }, true).time, 80.0f);
}

TEST(OpInstanceCharge, ConstantCurrentPhaseOnly) {
    EXPECT_FLOAT_EQ(chargeInstance().chargeDuration(0.0f, 80.0f), 80.0f);
}

TEST(OpInstanceCharge, BothPhasesAreTapered) {
    EXPECT_FLOAT_EQ(chargeInstance().chargeDuration(0.0f, 100.0f), 120.0f);
}

TEST(OpInstanceCharge, StartingAboveTheKneeUsesTheTaperedRateOnly) {
    EXPECT_FLOAT_EQ(chargeInstance().chargeDuration(90.0f, 100.0f), 20.0f);
}

TEST(OpInstanceCharge, NothingToChargeCostsNoTime) {
    EXPECT_FLOAT_EQ(chargeInstance().chargeDuration(50.0f, 50.0f), 0.0f);
}

TEST(OpInstanceStations, OnlyDeclaredNodesAreStations) {
    const des::OpInstance op = chargeInstance();

    EXPECT_TRUE(op.isStation(2));
    EXPECT_FALSE(op.isStation(0));
    EXPECT_FALSE(op.isStation(1));
}

TEST(OpInstanceStations, VisitingAStationRefillsToMaxEnergy) {
    const des::OpInstance op = chargeInstance();
    const des::OpInstance::Sim sim = op.simulateRoute({ 2 }, false);

    EXPECT_FLOAT_EQ(sim.socEnd, 100.0f);
}
