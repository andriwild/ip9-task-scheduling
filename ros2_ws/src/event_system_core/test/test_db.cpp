#include <gtest/gtest.h>
#include <QCoreApplication>

#include "../src/util/db.h"

static int argc = 1;
static char arg0[] = "test_db";
static char* argv[] = {arg0};

class DBTest : public ::testing::Test {
protected:
    std::unique_ptr<QCoreApplication> app;
    std::unique_ptr<DBClient> db;

    void SetUp() override {
        app = std::make_unique<QCoreApplication>(argc, argv);
        db = std::make_unique<DBClient>(DBConfig{"wsr_user", "wsr_password"});
    }
};

TEST_F(DBTest, ConnectionSucceeds) {
    EXPECT_TRUE(db->init());
}

TEST_F(DBTest, WaypointsReturnsData) {
    auto result = db->waypoints();
    ASSERT_TRUE(result.has_value());
    EXPECT_GT(result.value().size(), 0u);
}

TEST_F(DBTest, WaypointsHaveValidCoordinates) {
    auto result = db->waypoints();
    ASSERT_TRUE(result.has_value());

    for (const auto& loc : result.value()) {
        EXPECT_FALSE(loc.m_name.empty());
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
    auto result = db->areaByName("IMVS_CoffeeMachine");
    ASSERT_TRUE(result.has_value());
    EXPECT_GT(result.value(), 0.0);
}

TEST_F(DBTest, AreaByNameReturnsNulloptForUnknown) {
    auto result = db->areaByName("NonExistentZone");
    EXPECT_FALSE(result.has_value());
}
