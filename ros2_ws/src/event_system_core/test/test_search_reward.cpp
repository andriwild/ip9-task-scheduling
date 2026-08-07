#include <gtest/gtest.h>

#include "../src/algo/search/search_reward.h"

namespace {

const std::vector<std::string> kRooms = { "office", "kitchen" };
const std::vector<des::RoomType> kTypes = { des::RoomType::WORKPLACE, des::RoomType::KITCHEN };

des::SightingLog logWith(const int hits, const int misses, const std::string& room) {
    des::SightingLog log;
    for (int i = 0; i < hits; ++i) {
        log.add(des::Sighting{ i, "anna", room, des::SightingKind::PRESENT });
    }
    for (int i = 0; i < misses; ++i) {
        log.add(des::Sighting{ i, "anna", room, des::SightingKind::ABSENT });
    }
    return log;
}

float rewardOf(const std::vector<des::OpNode>& nodes, const std::string& room) {
    for (const auto& node : nodes) {
        if (node.name == room) {
            return node.reward;
        }
    }
    return -1.0f;
}

}  // namespace

static constexpr double kPriorWeight = 4.0;
static constexpr float kWorkplacePrior = 0.6f;

TEST(RoomProbability, WithoutEvidenceItIsThePrior) {
    EXPECT_NEAR(des::roomProbability(0, 0, 0.05f, kPriorWeight), 0.05f, 1e-6f);
    EXPECT_NEAR(des::roomProbability(0, 0, 0.60f, kPriorWeight), 0.60f, 1e-6f);
}

TEST(RoomProbability, OneHitPullsItUp) {
    EXPECT_NEAR(des::roomProbability(1, 0, 0.05f, kPriorWeight), 0.24f, 1e-6f);
}

TEST(RoomProbability, OneMissPullsItDown) {
    EXPECT_NEAR(des::roomProbability(0, 1, 0.05f, kPriorWeight), 0.04f, 1e-6f);
}

TEST(RoomProbability, MoreHitsNeverLowerIt) {
    EXPECT_LT(des::roomProbability(1, 0, 0.05f, kPriorWeight), des::roomProbability(2, 0, 0.05f, kPriorWeight));
    EXPECT_LT(des::roomProbability(2, 0, 0.05f, kPriorWeight), des::roomProbability(10, 0, 0.05f, kPriorWeight));
}

TEST(OccupancyPrior, TheWorkplaceAlwaysWins) {
    EXPECT_FLOAT_EQ(des::occupancyPrior(true, des::RoomType::OTHER, {}, false, kWorkplacePrior), 0.6f);
    EXPECT_FLOAT_EQ(des::occupancyPrior(true, des::RoomType::KITCHEN, {"Chef"}, true, kWorkplacePrior), 0.6f);
}

TEST(OccupancyPrior, WithoutRolePriorEveryOtherRoomIsFlat) {
    EXPECT_FLOAT_EQ(des::occupancyPrior(false, des::RoomType::KITCHEN, {"Chef"}, false, kWorkplacePrior), 0.05f);
    EXPECT_FLOAT_EQ(des::occupancyPrior(false, des::RoomType::TOILET, {"Chef"}, false, kWorkplacePrior), 0.05f);
}

TEST(OccupancyPrior, WithRolePriorTheRoleDecides) {
    EXPECT_FLOAT_EQ(des::occupancyPrior(false, des::RoomType::KITCHEN, {"Chef"}, true, kWorkplacePrior), 0.30f);
    EXPECT_FLOAT_EQ(des::occupancyPrior(false, des::RoomType::KITCHEN, {"Employee"}, true, kWorkplacePrior), 0.10f);
}

TEST(OccupancyProbability, UnseenRoomsKeepTheirPrior) {
    const auto nodes = des::occupancyProbability(des::SightingLog{}, "anna", "office", kRooms, kTypes, {}, false, kPriorWeight, kWorkplacePrior);

    ASSERT_EQ(nodes.size(), 2u);
    EXPECT_NEAR(rewardOf(nodes, "office"), 0.60f, 1e-6f);
    EXPECT_NEAR(rewardOf(nodes, "kitchen"), 0.05f, 1e-6f);
}

TEST(OccupancyProbability, SightingsRaiseTheRoomAboveItsPrior) {
    const auto nodes = des::occupancyProbability(logWith(2, 0, "kitchen"), "anna", "office", kRooms, kTypes, {}, false, kPriorWeight, kWorkplacePrior);

    EXPECT_NEAR(rewardOf(nodes, "kitchen"), 2.2f / 6.0f, 1e-6f);
    EXPECT_NEAR(rewardOf(nodes, "office"), 0.60f, 1e-6f);
}

TEST(OccupancyProbability, MissesPushTheWorkplaceBelowItsPrior) {
    const auto nodes = des::occupancyProbability(logWith(0, 3, "office"), "anna", "office", kRooms, kTypes, {}, false, kPriorWeight, kWorkplacePrior);

    EXPECT_LT(rewardOf(nodes, "office"), 0.60f);
}

TEST(FrequencyReward, OnlyRoomsWithHitsAreListed) {
    const auto nodes = des::frequencyReward(logWith(3, 0, "kitchen"), "anna", kRooms);

    ASSERT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes.front().name, "kitchen");
    EXPECT_FLOAT_EQ(nodes.front().reward, 3.0f);
}
