// Generated with Claude Code (Anthropic), then reviewed and adapted by the author. See the index of auxiliary tools.
#include <gtest/gtest.h>
#include <QCoreApplication>

#include "../src/io/db.h"

static int argc = 1;
static char arg0[] = "test_db";
static char* argv[] = {arg0};

class DBTest : public ::testing::Test {
protected:
    std::unique_ptr<QCoreApplication> app;
    std::unique_ptr<des::DBClient> db;

    void SetUp() override {
        app = std::make_unique<QCoreApplication>(argc, argv);
        db = std::make_unique<des::DBClient>(des::DBConfig{"wsr_user", "wsr_password"});
    }
};

TEST_F(DBTest, ConnectionSucceeds) {
    EXPECT_TRUE(db->init());
}

TEST_F(DBTest, RoomsReturnsData) {
    auto result = db->rooms();
    ASSERT_TRUE(result.has_value());
    EXPECT_GT(result.value().size(), 0u);
}

TEST_F(DBTest, RoomsHaveNamesMatchingTheirKey) {
    auto result = db->rooms();
    ASSERT_TRUE(result.has_value());

    for (const auto& [name, room] : result.value()) {
        EXPECT_FALSE(name.empty());
        EXPECT_EQ(room.m_name, name);
    }
}

TEST_F(DBTest, PersonByNameFindsKnownPerson) {
    auto result = db->personByName("Andri", "Wild");
    ASSERT_TRUE(result.has_value());

    auto& person = result.value();
    EXPECT_EQ(person.firstName, "Andri");
    EXPECT_EQ(person.lastName, "Wild");
}

TEST_F(DBTest, PersonByNameReturnsNulloptForUnknown) {
    auto result = db->personByName("Does", "NotExist");
    EXPECT_FALSE(result.has_value());
}

TEST_F(DBTest, AreaByNameReturnsPositiveArea) {
    auto result = db->areaByName("IMVS_Kitchen");
    ASSERT_TRUE(result.has_value());
    EXPECT_GT(result.value(), 0.0);
}

TEST_F(DBTest, AreaByNameReturnsNulloptForUnknown) {
    auto result = db->areaByName("NonExistentZone");
    EXPECT_FALSE(result.has_value());
}

TEST_F(DBTest, RoomsContainKnownZoneWithPositiveArea) {
    auto result = db->rooms();
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result.value().contains("IMVS_Kitchen"));
    const auto& area = result.value().at("IMVS_Kitchen").m_area;
    ASSERT_TRUE(area.has_value());
    EXPECT_GT(area.value(), 0.0);
}

TEST_F(DBTest, RoomAreaConsistentWithAreaByName) {
    auto all = db->rooms();
    auto single = db->areaByName("IMVS_Kitchen");
    ASSERT_TRUE(all.has_value());
    ASSERT_TRUE(single.has_value());
    ASSERT_TRUE(all.value().at("IMVS_Kitchen").m_area.has_value());
    EXPECT_DOUBLE_EQ(all.value().at("IMVS_Kitchen").m_area.value(), single.value());
}

TEST_F(DBTest, FootprintsAreOpenRingsWithAtLeastThreeVertices) {
    auto result = db->rooms();
    ASSERT_TRUE(result.has_value());
    for (const auto& [name, room] : result.value()) {
        if (room.m_footprint.empty()) {
            continue;
        }
        const auto& ring = room.m_footprint;
        EXPECT_GE(ring.size(), 3u) << name;
        const bool closed = ring.front().m_x == ring.back().m_x && ring.front().m_y == ring.back().m_y;
        EXPECT_FALSE(closed) << name;
    }
}

TEST_F(DBTest, FootprintAndAreaAppearTogether) {
    auto result = db->rooms();
    ASSERT_TRUE(result.has_value());
    for (const auto& [name, room] : result.value()) {
        EXPECT_EQ(room.m_footprint.empty(), !room.m_area.has_value()) << name;
    }
}
