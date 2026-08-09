#include <gtest/gtest.h>

#include "../src/algo/search/role_prior.h"

namespace {

const des::RolePrior& chef() {
    static const des::RolePrior entry = des::rolePriors().back();
    return entry;
}

}  // namespace

TEST(PriorFor, MapsEveryRoomTypeToItsOwnColumn) {
    EXPECT_FLOAT_EQ(des::priorFor(chef(), des::RoomType::KITCHEN), 0.30f);
    EXPECT_FLOAT_EQ(des::priorFor(chef(), des::RoomType::WORKPLACE), 0.03f);
    EXPECT_FLOAT_EQ(des::priorFor(chef(), des::RoomType::CLASSROOM), 0.01f);
    EXPECT_FLOAT_EQ(des::priorFor(chef(), des::RoomType::MEETING), 0.01f);
    EXPECT_FLOAT_EQ(des::priorFor(chef(), des::RoomType::OTHER), 0.06f);
    EXPECT_FLOAT_EQ(des::priorFor(chef(), des::RoomType::TOILET), 0.02f);
}

TEST(RolePrior, KnownRoleUsesItsTableEntry) {
    EXPECT_FLOAT_EQ(des::rolePrior({"Chef"}, des::RoomType::KITCHEN, 0.05f), 0.30f);
}

TEST(RolePrior, StudentsAreLessLikelyInMeetingsThanTeachers) {
    const float student = des::rolePrior({"Student"}, des::RoomType::MEETING, 0.05f);
    const float teacher = des::rolePrior({"Teacher"}, des::RoomType::MEETING, 0.05f);
    EXPECT_LT(student, teacher);
}

TEST(RolePrior, SeveralRolesTakeTheHighestValue) {
    EXPECT_FLOAT_EQ(des::rolePrior({"Student", "Chef"}, des::RoomType::CLASSROOM, 0.05f), 0.12f);
    EXPECT_FLOAT_EQ(des::rolePrior({"Student", "Chef"}, des::RoomType::KITCHEN, 0.05f), 0.30f);
}

TEST(RolePrior, UnknownRoleFallsBack) {
    EXPECT_FLOAT_EQ(des::rolePrior({"Janitor"}, des::RoomType::KITCHEN, 0.05f), 0.05f);
}

TEST(RolePrior, NoRoleAtAllFallsBack) {
    EXPECT_FLOAT_EQ(des::rolePrior({}, des::RoomType::KITCHEN, 0.05f), 0.05f);
}

TEST(RolePrior, AKnownRoleWinsOverTheFallback) {
    EXPECT_FLOAT_EQ(des::rolePrior({"Janitor", "Chef"}, des::RoomType::KITCHEN, 0.05f), 0.30f);
}
