#include <gtest/gtest.h>

#include "../src/algo/search/search_reward.h"

namespace {

const std::vector<std::string> kRooms = { "office", "kitchen" };
const std::vector<des::RoomType> kTypes = { des::RoomType::WORKPLACE, des::RoomType::KITCHEN };

SightingLog logWith(const int hits, const int misses, const std::string& room) {
    SightingLog log;
    for (int i = 0; i < hits; ++i) {
        log.add(Sighting{ i, "anna", room, SightingKind::PRESENT });
    }
    for (int i = 0; i < misses; ++i) {
        log.add(Sighting{ i, "anna", room, SightingKind::ABSENT });
    }
    return log;
}

float rewardOf(const std::vector<OpNode>& nodes, const std::string& room) {
    for (const auto& node : nodes) {
        if (node.name == room) {
            return node.reward;
        }
    }
    return -1.0f;
}

}  // namespace

TEST(RoomProbability, WithoutEvidenceItIsThePrior) {
    EXPECT_NEAR(roomProbability(0, 0, 0.05f), 0.05f, 1e-6f);
    EXPECT_NEAR(roomProbability(0, 0, 0.60f), 0.60f, 1e-6f);
}

TEST(RoomProbability, OneHitPullsItUp) {
    EXPECT_NEAR(roomProbability(1, 0, 0.05f), 0.24f, 1e-6f);
}

TEST(RoomProbability, OneMissPullsItDown) {
    EXPECT_NEAR(roomProbability(0, 1, 0.05f), 0.04f, 1e-6f);
}

TEST(RoomProbability, MoreHitsNeverLowerIt) {
    EXPECT_LT(roomProbability(1, 0, 0.05f), roomProbability(2, 0, 0.05f));
    EXPECT_LT(roomProbability(2, 0, 0.05f), roomProbability(10, 0, 0.05f));
}

TEST(OccupancyPrior, TheWorkplaceAlwaysWins) {
    EXPECT_FLOAT_EQ(occupancyPrior(true, des::RoomType::OTHER, {}, false), 0.6f);
    EXPECT_FLOAT_EQ(occupancyPrior(true, des::RoomType::KITCHEN, {"Chef"}, true), 0.6f);
}

TEST(OccupancyPrior, WithoutRolePriorEveryOtherRoomIsFlat) {
    EXPECT_FLOAT_EQ(occupancyPrior(false, des::RoomType::KITCHEN, {"Chef"}, false), 0.05f);
    EXPECT_FLOAT_EQ(occupancyPrior(false, des::RoomType::TOILET, {"Chef"}, false), 0.05f);
}

TEST(OccupancyPrior, WithRolePriorTheRoleDecides) {
    EXPECT_FLOAT_EQ(occupancyPrior(false, des::RoomType::KITCHEN, {"Chef"}, true), 0.30f);
    EXPECT_FLOAT_EQ(occupancyPrior(false, des::RoomType::KITCHEN, {"Employee"}, true), 0.10f);
}

TEST(OccupancyProbability, UnseenRoomsKeepTheirPrior) {
    const auto nodes = occupancyProbability(SightingLog{}, "anna", "office", kRooms, kTypes, {}, false);

    ASSERT_EQ(nodes.size(), 2u);
    EXPECT_NEAR(rewardOf(nodes, "office"), 0.60f, 1e-6f);
    EXPECT_NEAR(rewardOf(nodes, "kitchen"), 0.05f, 1e-6f);
}

TEST(OccupancyProbability, SightingsRaiseTheRoomAboveItsPrior) {
    const auto nodes = occupancyProbability(logWith(2, 0, "kitchen"), "anna", "office", kRooms, kTypes, {}, false);

    EXPECT_NEAR(rewardOf(nodes, "kitchen"), 2.2f / 6.0f, 1e-6f);
    EXPECT_NEAR(rewardOf(nodes, "office"), 0.60f, 1e-6f);
}

TEST(OccupancyProbability, MissesPushTheWorkplaceBelowItsPrior) {
    const auto nodes = occupancyProbability(logWith(0, 3, "office"), "anna", "office", kRooms, kTypes, {}, false);

    EXPECT_LT(rewardOf(nodes, "office"), 0.60f);
}

TEST(FrequencyReward, OnlyRoomsWithHitsAreListed) {
    const auto nodes = frequencyReward(logWith(3, 0, "kitchen"), "anna", kRooms);

    ASSERT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes.front().name, "kitchen");
    EXPECT_FLOAT_EQ(nodes.front().reward, 3.0f);
}
