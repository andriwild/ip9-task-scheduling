// Generated with Claude Code (Anthropic), then reviewed and adapted by the author. See the index of auxiliary tools.
#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>

#include "../src/plugins/clean/clean_plugin.h"
#include "../src/plugins/clean/clean_order.h"
#include "../src/plugins/data_acquisition/data_acquisition_plugin.h"
#include "../src/plugins/data_acquisition/data_acquisition_order.h"

namespace {

constexpr int kDay = 86400;

class FakeWorld: public des::IWorldModel {
    std::map<std::string, des::Room> m_rooms;
    std::map<std::pair<std::string, std::string>, int> m_serviced;
public:
    void addRoom(const std::string& name, const std::optional<double> area) {
        m_rooms.emplace(name, des::Room(name, des::Point(0.0, 0.0, 0.0), area));
    }
    const des::Room& room(const std::string& name) const override {
        return m_rooms.at(name);
    }
    std::vector<std::string> roomNames() const override {
        std::vector<std::string> names;
        for (const auto& [name, r] : m_rooms) {
            names.push_back(name);
        }
        return names;
    }
    std::optional<int> lastServiced(const std::string& room, const std::string& type) const override {
        const auto it = m_serviced.find({room, type});
        if (it == m_serviced.end()) {
            return std::nullopt;
        }
        return it->second;
    }
    void recordServiced(const std::string& room, const std::string& type, const int time) override {
        m_serviced[{room, type}] = time;
    }
};

class FakeClock: public des::ISimClock {
    int m_now = 0;
public:
    void set(const int now) {
        m_now = now;
    }
    int getTime() const override {
        return m_now;
    }
    std::optional<int> getSimulationEndTime() const override {
        return std::nullopt;
    }
};

struct Fixture {
    FakeWorld world;
    FakeClock clock;
    des::SimConfig cfg{};
    des::CleanPlugin clean;
    des::DataAcquisition acquisition;

    Fixture() {
        clean.loadConfig({{"value_per_sqm", 0.3}, {"cleaning_interval", 86400.0}});
        acquisition.loadConfig({{"mission_value", 3.0}, {"data_acquisition_interval", 86400.0}});
    }

    des::EstimationView view() const {
        return des::EstimationView{world, clock, cfg};
    }

    double cleanReward(const std::string& room) {
        des::CleanOrder o;
        o.id = 1;
        o.type = "clean";
        o.roomName = room;
        return clean.estimateReward(o, view());
    }

    double acquisitionReward(const std::string& room) {
        des::DataAcquisitionOrder o;
        o.id = 2;
        o.type = "data_acquisition";
        o.roomName = room;
        return acquisition.estimateReward(o, view());
    }
};

}  // namespace

TEST(MissionReward, CleanAtNominalDueIsAreaTimesRate) {
    Fixture f;
    f.world.addRoom("median", 26.8);
    f.world.recordServiced("median", "clean", 0);
    f.clock.set(kDay);

    EXPECT_NEAR(f.cleanReward("median"), 0.3 * 26.8, 1e-9);
}

TEST(MissionReward, AcquisitionAtNominalDueIsItsValue) {
    Fixture f;
    f.world.addRoom("median", 26.8);
    f.world.recordServiced("median", "data_acquisition", 0);
    f.clock.set(kDay);

    EXPECT_NEAR(f.acquisitionReward("median"), 3.0, 1e-9);
}

TEST(MissionReward, AcquisitionIgnoresRoomSize) {
    Fixture f;
    f.world.addRoom("small", 3.6);
    f.world.addRoom("huge", 295.5);
    f.world.recordServiced("small", "data_acquisition", 0);
    f.world.recordServiced("huge", "data_acquisition", 0);
    f.clock.set(kDay);

    EXPECT_NEAR(f.acquisitionReward("small"), f.acquisitionReward("huge"), 1e-9);
}

TEST(MissionReward, BacklogKeepsEscalatingInsteadOfSaturating) {
    Fixture f;
    f.world.addRoom("median", 26.8);
    f.world.recordServiced("median", "clean", 0);
    f.world.recordServiced("median", "data_acquisition", 0);

    f.clock.set(kDay);
    const double cleanDue = f.cleanReward("median");
    const double acqDue   = f.acquisitionReward("median");

    f.clock.set(5 * kDay);
    EXPECT_NEAR(f.cleanReward("median"), 5.0 * cleanDue, 1e-9);
    EXPECT_NEAR(f.acquisitionReward("median"), 5.0 * acqDue, 1e-9);
}

TEST(MissionReward, JustServicedIsWorthless) {
    Fixture f;
    f.world.addRoom("median", 26.8);
    f.world.recordServiced("median", "clean", 4000);
    f.world.recordServiced("median", "data_acquisition", 4000);
    f.clock.set(4000);

    EXPECT_NEAR(f.cleanReward("median"), 0.0, 1e-9);
    EXPECT_NEAR(f.acquisitionReward("median"), 0.0, 1e-9);
}

TEST(MissionReward, NeverServicedCountsAsExactlyDue) {
    Fixture f;
    f.world.addRoom("median", 26.8);
    f.clock.set(12345);

    EXPECT_NEAR(f.cleanReward("median"), 0.3 * 26.8, 1e-9);
    EXPECT_NEAR(f.acquisitionReward("median"), 3.0, 1e-9);
}

TEST(MissionReward, LargeRoomsAreNoLongerCapped) {
    Fixture f;
    f.world.addRoom("small", 50.0);
    f.world.addRoom("huge", 295.5);
    f.world.recordServiced("small", "clean", 0);
    f.world.recordServiced("huge", "clean", 0);
    f.clock.set(kDay);

    EXPECT_NEAR(f.cleanReward("huge") / f.cleanReward("small"), 295.5 / 50.0, 1e-9);
}

TEST(MissionReward, RewardPerServiceSecondBarelyDependsOnRoomSize) {
    Fixture f;
    f.world.addRoom("small", 12.7);
    f.world.addRoom("large", 79.3);
    f.world.recordServiced("small", "clean", 0);
    f.world.recordServiced("large", "clean", 0);
    f.clock.set(kDay);

    const double cleaningArea = f.clean.config().cleaningArea;
    const auto density = [&](const std::string& room, const double area) {
        const double seconds = (area / cleaningArea + 1.0) * 2.0 * std::sqrt(cleaningArea);
        return f.cleanReward(room) / seconds;
    };

    EXPECT_NEAR(density("large", 79.3) / density("small", 12.7), 1.0, 0.01);
}

TEST(MissionReward, RoomWithoutAreaIsWorthNothing) {
    Fixture f;
    f.world.addRoom("unknown", std::nullopt);
    f.world.recordServiced("unknown", "clean", 0);
    f.clock.set(kDay);

    EXPECT_NEAR(f.cleanReward("unknown"), 0.0, 1e-9);
}
