#include <gtest/gtest.h>

#include "model/battery.hpp"

namespace {

constexpr double kCapacity = 100.0;  // Ah
constexpr double kLow      = 20.0;   // %
constexpr double kFull     = 100.0;  // %
constexpr double kVoltage  = 12.0;   // V
constexpr double kPower    = 500.0;  // W

constexpr double kCvThreshold   = 0.85;
constexpr double kTaperFraction = 0.5;

Battery makeBattery(const double initialCapacity, const bool chargeToFull) {
    return Battery(kCapacity, initialCapacity, kLow, kFull, kVoltage,
                   kCvThreshold, kTaperFraction, chargeToFull);
}

}  // namespace

TEST(Battery, PartialChargeIsFasterThanFullCharge) {
    const double startCapacity = 10.0;
    const double partial = makeBattery(startCapacity, false).timeToFull(kPower);
    const double full    = makeBattery(startCapacity, true).timeToFull(kPower);

    EXPECT_GT(partial, 0.0);
    EXPECT_GT(full, 0.0);
    EXPECT_LT(partial, full);
}

TEST(Battery, PhaseTransitionOnlyWhenChargingIntoPhaseTwo) {
    EXPECT_GE(makeBattery(10.0, true).timeToPhaseTransition(kPower), 0.0);

    EXPECT_DOUBLE_EQ(makeBattery(10.0, false).timeToPhaseTransition(kPower), -1.0);

    EXPECT_DOUBLE_EQ(makeBattery(90.0, true).timeToPhaseTransition(kPower), -1.0);
}

TEST(Battery, TransitionTimePlusTaperedPhaseEqualsTotal) {
    Battery bat = makeBattery(10.0, true);
    const double transition = bat.timeToPhaseTransition(kPower);
    const double total      = bat.timeToFull(kPower);

    EXPECT_GT(transition, 0.0);
    EXPECT_GT(total, transition);
}

TEST(Battery, IsFullyChargedReflectsTarget) {
    EXPECT_FALSE(makeBattery(10.0, true).isFullyCharged());
    EXPECT_TRUE(makeBattery(kCapacity, true).isFullyCharged());
}

TEST(Battery, IsFullyChargedHonorsChargeToFull) {
    const double cvCapacity = kCvThreshold * kCapacity;
    EXPECT_TRUE(makeBattery(cvCapacity, false).isFullyCharged());
    EXPECT_FALSE(makeBattery(cvCapacity, true).isFullyCharged());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
