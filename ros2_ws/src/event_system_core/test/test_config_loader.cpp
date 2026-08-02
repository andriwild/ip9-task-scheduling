#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <memory>

#include "../src/init/config_loader.h"
#include "../src/plugins/accompany/accompany_order.h"
#include "../src/plugins/accompany/accompany_plugin.h"
#include "../src/plugins/clean/clean_plugin.h"
#include "../src/plugins/data_acquisition/data_acquisition_plugin.h"
#include "../src/plugins/information/information_plugin.h"
#include "../src/plugins/order_registry.h"

namespace {

std::string fixturesDir() {
    return std::string(TEST_FIXTURES_DIR);
}

std::shared_ptr<AccompanyOrder> makeAccompanyOrder(
        const std::string& person,
        const std::string& room,
        int appointmentTime = 36000,
        const std::string& description = "Test") {
    auto o = std::make_shared<AccompanyOrder>();
    o->type = "accompany";
    o->personName = person;
    o->roomName = room;
    o->deadline = appointmentTime;
    o->description = description;
    return o;
}

// Register all plugins once so loadSimConfig/saveSimConfig iterate over the
// full set (and the nested sub-objects round-trip cleanly).
class PluginRegistry : public ::testing::Environment {
public:
    void SetUp() override {
        static bool registered = false;
        if (!registered) {
            OrderRegistry::instance().registerPlugin(std::make_unique<AccompanyOrderPlugin>());
            OrderRegistry::instance().registerPlugin(std::make_unique<CleanPlugin>());
            OrderRegistry::instance().registerPlugin(std::make_unique<DataAcquisition>());
            OrderRegistry::instance().registerPlugin(std::make_unique<InformationPlugin>());
            registered = true;
        }
    }
};
::testing::Environment* const kRegistryEnv = ::testing::AddGlobalTestEnvironment(new PluginRegistry);

} // namespace

// --- loadOrderConfig ---

TEST(ConfigLoaderOrders, LoadsValidOrders) {
    auto result = ConfigLoader::loadOrderConfig(fixturesDir() + "/test_appointments.json");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2u);

    auto first = std::dynamic_pointer_cast<AccompanyOrder>((*result)[0]);
    ASSERT_NE(first, nullptr);
    EXPECT_EQ(first->personName, "Max");
    EXPECT_EQ(first->roomName, "5.2B10");
    EXPECT_EQ(first->deadline.value(), 36000);
    EXPECT_EQ(first->description, "Meeting A");
    EXPECT_EQ(first->id, 0);

    auto second = std::dynamic_pointer_cast<AccompanyOrder>((*result)[1]);
    ASSERT_NE(second, nullptr);
    EXPECT_EQ(second->personName, "Anna");
    EXPECT_EQ(second->id, 1);
}

TEST(ConfigLoaderOrders, DefaultStateIsPending) {
    auto result = ConfigLoader::loadOrderConfig(fixturesDir() + "/test_appointments.json");
    ASSERT_TRUE(result.has_value());
    for (const auto& order : *result) {
        EXPECT_EQ(order->state, des::MissionState::PENDING);
    }
}

// --- loadEmployees ---

TEST(ConfigLoaderEmployees, LoadsValidEmployees) {
    auto result = ConfigLoader::loadEmployees(fixturesDir() + "/test_employees.json");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2u);

    EXPECT_EQ((*result)[0]->firstName, "Max");
    EXPECT_EQ((*result)[0]->lastName, "Mustermann");
    EXPECT_EQ((*result)[0]->workplace, "5.2B03");
    EXPECT_EQ((*result)[0]->roomLabels.size(), 3u);
    EXPECT_EQ((*result)[0]->transitionMatrix.size(), 3u);
    EXPECT_EQ((*result)[0]->transitionMatrix[0].size(), 3u);
}

TEST(ConfigLoaderEmployees, NonexistentFileReturnsNullopt) {
    auto result = ConfigLoader::loadEmployees("/tmp/nonexistent_xyz_12345.json");
    EXPECT_FALSE(result.has_value());
}

TEST(ConfigLoaderEmployees, EmployeeFieldsParsedCorrectly) {
    auto result = ConfigLoader::loadEmployees(fixturesDir() + "/test_employees.json");
    ASSERT_TRUE(result.has_value());

    const auto& anna = (*result)[1];
    EXPECT_EQ(anna->id, 2);
    EXPECT_EQ(anna->firstName, "Anna");
    EXPECT_EQ(anna->lastName, "Schmidt");
    EXPECT_EQ(anna->birthDate, "1993-11-02");
    EXPECT_EQ(anna->sex, "female");
    EXPECT_EQ(anna->color, "#00A000");
    EXPECT_EQ(anna->workplace, "5.2B01");
}

// --- loadSimConfig ---

TEST(ConfigLoaderSimConfig, LoadsValidConfig) {
    auto result = ConfigLoader::loadSimConfig(fixturesDir() + "/test_sim_config.json");
    ASSERT_TRUE(result.has_value());

    EXPECT_DOUBLE_EQ(result->robotSpeed, 0.5);
    EXPECT_DOUBLE_EQ(result->timeBuffer, 60.0);
    EXPECT_DOUBLE_EQ(result->batteryCapacity, 100.0);
    EXPECT_DOUBLE_EQ(result->initialBatteryCapacity, 80.0);
    EXPECT_DOUBLE_EQ(result->chargingRate, 0.5);
    EXPECT_DOUBLE_EQ(result->lowBatteryThreshold, 20.0);
    EXPECT_DOUBLE_EQ(result->fullBatteryThreshold, 95.0);
    EXPECT_EQ(result->dockLocation, "IMVS_Dock");
    EXPECT_FALSE(result->cacheEnabled);
    EXPECT_TRUE(std::filesystem::path(result->appointmentsPath).is_absolute());
    EXPECT_EQ(std::filesystem::path(result->appointmentsPath).filename(), "test.json");

    // Plugin-owned parameters live under their typeName() sub-object now.
    EXPECT_DOUBLE_EQ(accompanyConfig().accompanySpeed, 0.3);
    EXPECT_DOUBLE_EQ(accompanyConfig().conversationProbability, 0.5);
    EXPECT_DOUBLE_EQ(accompanyConfig().conversationDurationMean, 30.0);
    EXPECT_DOUBLE_EQ(accompanyConfig().conversationDurationStd, 10.0);
    EXPECT_DOUBLE_EQ(accompanyConfig().appointmentDuration, 1800.0);
    EXPECT_DOUBLE_EQ(cleanConfig().cleaningArea, 0.09);
    EXPECT_DOUBLE_EQ(dataAcquisitionConfig().dataAcquisitionDuration, 120.0);
    EXPECT_DOUBLE_EQ(informationConfig().informationDurationMin, 30.0);
    EXPECT_DOUBLE_EQ(informationConfig().informationDurationMax, 90.0);
}

TEST(ConfigLoaderSimConfig, DistributionTypesParsedCorrectly) {
    auto result = ConfigLoader::loadSimConfig(fixturesDir() + "/test_sim_config.json");
    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(result->arrivalDistribution, des::DistributionType::NORMAL);
    EXPECT_EQ(result->departureDistribution, des::DistributionType::NORMAL);
    EXPECT_DOUBLE_EQ(result->arrivalMean, 32400.0);
    EXPECT_DOUBLE_EQ(result->arrivalStd, 3600.0);
}

TEST(ConfigLoaderSimConfig, NonexistentFileReturnsNullopt) {
    auto result = ConfigLoader::loadSimConfig("/tmp/nonexistent_xyz_12345.json");
    EXPECT_FALSE(result.has_value());
}

TEST(ConfigLoaderSimConfig, SeedAndRoundModeFallBackToDefaults) {
    auto result = ConfigLoader::loadSimConfig(fixturesDir() + "/test_sim_config.json");
    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(result->seed, 42u);
    EXPECT_EQ(result->roundMode, des::RoundMode::REPLICATION);
    EXPECT_TRUE(result->missionTraceRounds.empty());
    EXPECT_TRUE(result->missionTraceWindow.empty());
}

TEST(ConfigLoaderSimConfig, OverrideSetsSeedAndRoundMode) {
    const std::string overridePath = "/tmp/test_seed_override.json";
    std::ofstream out(overridePath);
    out << R"({"seed": 7, "round_mode": "continuation", "mission_trace_rounds": [1, 3], "mission_trace_window": [0, 86400]})";
    out.close();

    auto result = ConfigLoader::loadSimConfig(fixturesDir() + "/test_sim_config.json", overridePath);
    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(result->seed, 7u);
    EXPECT_EQ(result->roundMode, des::RoundMode::CONTINUATION);
    EXPECT_EQ(result->missionTraceRounds, (std::vector<int>{1, 3}));
    EXPECT_EQ(result->missionTraceWindow, (std::vector<int>{0, 86400}));
    EXPECT_DOUBLE_EQ(result->robotSpeed, 0.5);

    std::filesystem::remove(overridePath);
}

TEST(ConfigLoaderSimConfig, PathResolutionIsIdempotent) {
    const std::string relative = ConfigLoader::resolvePath("scenarios/single_accompany.json");
    EXPECT_TRUE(std::filesystem::path(relative).is_absolute());
    EXPECT_EQ(ConfigLoader::resolvePath(relative), relative);
}

TEST(ConfigLoaderSimConfig, MalformedTraceWindowReturnsNullopt) {
    const std::string overridePath = "/tmp/test_bad_window_override.json";
    std::ofstream out(overridePath);
    out << R"({"mission_trace_window": [100]})";
    out.close();

    auto result = ConfigLoader::loadSimConfig(fixturesDir() + "/test_sim_config.json", overridePath);
    EXPECT_FALSE(result.has_value());

    std::filesystem::remove(overridePath);
}

// --- saveSimConfig roundtrip ---

TEST(ConfigLoaderSimConfig, SaveAndReloadProducesSameConfig) {
    auto original = ConfigLoader::loadSimConfig(fixturesDir() + "/test_sim_config.json");
    ASSERT_TRUE(original.has_value());

    // Snapshot plugin-owned values right after load — the plugins are
    // singletons, so the second load below would overwrite them and we'd
    // lose the "original" to compare against.
    const auto accompanyOriginal       = accompanyConfig();
    const auto cleanOriginal           = cleanConfig();
    const auto dataAcquisitionOriginal = dataAcquisitionConfig();
    const auto informationOriginal     = informationConfig();

    std::string tmpFile = "/tmp/test_sim_config_roundtrip.json";
    auto configPtr = std::make_shared<des::SimConfig>(*original);
    ASSERT_TRUE(ConfigLoader::saveSimConfig(tmpFile, configPtr));

    auto reloaded = ConfigLoader::loadSimConfig(tmpFile);
    ASSERT_TRUE(reloaded.has_value());

    EXPECT_DOUBLE_EQ(original->robotSpeed, reloaded->robotSpeed);
    EXPECT_DOUBLE_EQ(original->timeBuffer, reloaded->timeBuffer);
    EXPECT_DOUBLE_EQ(original->batteryCapacity, reloaded->batteryCapacity);
    EXPECT_DOUBLE_EQ(original->initialBatteryCapacity, reloaded->initialBatteryCapacity);
    EXPECT_DOUBLE_EQ(original->chargingRate, reloaded->chargingRate);
    EXPECT_DOUBLE_EQ(original->lowBatteryThreshold, reloaded->lowBatteryThreshold);
    EXPECT_DOUBLE_EQ(original->fullBatteryThreshold, reloaded->fullBatteryThreshold);
    EXPECT_DOUBLE_EQ(original->energyConsumptionDrive, reloaded->energyConsumptionDrive);
    EXPECT_DOUBLE_EQ(original->energyConsumptionBase, reloaded->energyConsumptionBase);
    EXPECT_DOUBLE_EQ(original->driveTimeStd, reloaded->driveTimeStd);
    EXPECT_EQ(original->dockLocation, reloaded->dockLocation);
    EXPECT_EQ(original->cacheEnabled, reloaded->cacheEnabled);
    EXPECT_EQ(original->appointmentsPath, reloaded->appointmentsPath);
    EXPECT_EQ(original->arrivalDistribution, reloaded->arrivalDistribution);
    EXPECT_EQ(original->departureDistribution, reloaded->departureDistribution);

    // Plugin-owned values survive the round-trip too.
    EXPECT_DOUBLE_EQ(accompanyOriginal.accompanySpeed,           accompanyConfig().accompanySpeed);
    EXPECT_DOUBLE_EQ(accompanyOriginal.conversationProbability,  accompanyConfig().conversationProbability);
    EXPECT_DOUBLE_EQ(accompanyOriginal.conversationDurationMean, accompanyConfig().conversationDurationMean);
    EXPECT_DOUBLE_EQ(accompanyOriginal.conversationDurationStd,  accompanyConfig().conversationDurationStd);
    EXPECT_DOUBLE_EQ(accompanyOriginal.appointmentDuration,      accompanyConfig().appointmentDuration);
    EXPECT_DOUBLE_EQ(cleanOriginal.cleaningArea,                 cleanConfig().cleaningArea);
    EXPECT_DOUBLE_EQ(dataAcquisitionOriginal.dataAcquisitionDuration, dataAcquisitionConfig().dataAcquisitionDuration);
    EXPECT_DOUBLE_EQ(informationOriginal.informationDurationMin, informationConfig().informationDurationMin);
    EXPECT_DOUBLE_EQ(informationOriginal.informationDurationMax, informationConfig().informationDurationMax);

    std::filesystem::remove(tmpFile);
}

TEST(ConfigLoaderSimConfig, SaveToInvalidPathReturnsFalse) {
    auto config = std::make_shared<des::SimConfig>();
    EXPECT_FALSE(ConfigLoader::saveSimConfig("/nonexistent_dir/file.json", config));
}

// --- filterByAppointments ---

TEST(ConfigLoaderFilter, FiltersEmployeesByAccompanyOrder) {
    auto employees = ConfigLoader::loadEmployees(fixturesDir() + "/test_employees.json");
    ASSERT_TRUE(employees.has_value());

    des::OrderList orders = { makeAccompanyOrder("Max", "5.2B10") };

    auto filtered = ConfigLoader::filterByAppointments(*employees, orders);
    ASSERT_EQ(filtered.size(), 1u);
    EXPECT_EQ(filtered[0]->firstName, "Max");
}

TEST(ConfigLoaderFilter, NoMatchingEmployeesReturnsEmpty) {
    auto employees = ConfigLoader::loadEmployees(fixturesDir() + "/test_employees.json");
    ASSERT_TRUE(employees.has_value());

    des::OrderList orders = { makeAccompanyOrder("UnknownPerson", "RoomX") };

    auto filtered = ConfigLoader::filterByAppointments(*employees, orders);
    EXPECT_TRUE(filtered.empty());
}

TEST(ConfigLoaderFilter, AllEmployeesMatchedWhenAllHaveOrders) {
    auto employees = ConfigLoader::loadEmployees(fixturesDir() + "/test_employees.json");
    ASSERT_TRUE(employees.has_value());

    des::OrderList orders;
    for (const auto& emp : *employees) {
        orders.push_back(makeAccompanyOrder(emp->firstName, "Room"));
    }

    auto filtered = ConfigLoader::filterByAppointments(*employees, orders);
    EXPECT_EQ(filtered.size(), employees->size());
}

// --- validateConfig ---

TEST(ConfigLoaderValidation, ValidConfigDoesNotThrow) {
    auto employees = ConfigLoader::loadEmployees(fixturesDir() + "/test_employees.json");
    auto orders = ConfigLoader::loadOrderConfig(fixturesDir() + "/test_appointments.json");
    ASSERT_TRUE(employees.has_value());
    ASSERT_TRUE(orders.has_value());

    des::RoomMap locationMap;
    locationMap.emplace("5.2B03", des::Room("5.2B03", des::Point(0, 0, 0)));
    locationMap.emplace("5.2B01", des::Room("5.2B01", des::Point(1, 1, 0)));
    locationMap.emplace("5.2B10", des::Room("5.2B10", des::Point(2, 2, 0)));
    locationMap.emplace("5.2B_Elevator", des::Room("5.2B_Elevator", des::Point(3, 3, 0)));

    EXPECT_NO_THROW(ConfigLoader::validateConfig(*orders, *employees, locationMap, "5.2B_Elevator"));
}

TEST(ConfigLoaderValidation, UnknownPersonInAccompanyOrderThrows) {
    des::PersonList employees;
    auto emp = std::make_unique<des::Person>();
    emp->firstName = "Max";
    emp->roomLabels = {"RoomA"};
    emp->transitionMatrix = {{1.0}};
    employees.push_back(std::move(emp));

    des::OrderList orders = { makeAccompanyOrder("UnknownPerson", "RoomA") };

    des::RoomMap locationMap;
    locationMap.emplace("RoomA", des::Room("RoomA", des::Point(0, 0, 0)));

    EXPECT_THROW(ConfigLoader::validateConfig(orders, employees, locationMap, "RoomA"), std::runtime_error);
}

TEST(ConfigLoaderValidation, UnknownRoomInAccompanyOrderThrows) {
    auto emp = std::make_unique<des::Person>();
    emp->firstName = "Max";
    emp->roomLabels = {"RoomA"};
    emp->transitionMatrix = {{1.0}};

    des::OrderList orders = { makeAccompanyOrder("Max", "NonexistentRoom") };

    des::RoomMap locationMap;
    locationMap.emplace("RoomA", des::Room("RoomA", des::Point(0, 0, 0)));

    des::PersonList employees;
    employees.push_back(std::move(emp));
    EXPECT_THROW(
        ConfigLoader::validateConfig(orders, employees, locationMap, "RoomA"),
        std::runtime_error
    );
}

TEST(ConfigLoaderValidation, MismatchedTransitionMatrixThrows) {
    auto emp = std::make_unique<des::Person>();
    emp->firstName = "Max";
    emp->roomLabels = {"RoomA", "RoomB"};
    emp->transitionMatrix = {{1.0}}; // 1x1 but should be 2x2

    des::RoomMap locationMap;
    locationMap.emplace("RoomA", des::Room("RoomA", des::Point(0, 0, 0)));
    locationMap.emplace("RoomB", des::Room("RoomB", des::Point(1, 1, 0)));

    des::PersonList employees;
    employees.push_back(std::move(emp));
    EXPECT_THROW(
        ConfigLoader::validateConfig({}, employees, locationMap, "RoomA"),
        std::runtime_error
    );
}

TEST(ConfigLoaderBuildingSnapshot, LoadsFootprintsAndAreas) {
    auto map = ConfigLoader::loadBuildingSnapshot(fixturesDir() + "/test_building.json");
    ASSERT_TRUE(map.has_value());
    ASSERT_EQ(map->size(), 3u);

    const auto& roomA = map->at("RoomA");
    ASSERT_TRUE(roomA.m_area.has_value());
    EXPECT_DOUBLE_EQ(roomA.m_area.value(), 12.0);
    ASSERT_EQ(roomA.m_footprint.size(), 4u);
    EXPECT_DOUBLE_EQ(roomA.m_footprint[0].m_x, 0.0);
    EXPECT_DOUBLE_EQ(roomA.m_footprint[2].m_x, 4.0);
    EXPECT_DOUBLE_EQ(roomA.m_footprint[2].m_y, 3.0);

    const auto& roomB = map->at("RoomB");
    EXPECT_FALSE(roomB.m_area.has_value());
    EXPECT_TRUE(roomB.m_footprint.empty());
}

TEST(ConfigLoaderBuildingSnapshot, LoadsLegacySnapshotWithoutFootprints) {
    auto map = ConfigLoader::loadBuildingSnapshot(fixturesDir() + "/test_building_legacy.json");
    ASSERT_TRUE(map.has_value());
    const auto& roomA = map->at("RoomA");
    ASSERT_TRUE(roomA.m_area.has_value());
    EXPECT_TRUE(roomA.m_footprint.empty());
}

TEST(ConfigLoaderSimConfig, SaveMergesUnknownKeysIntoExistingFile) {
    const std::string path = "/tmp/des_test_sim_config_merge.json";
    {
        std::ofstream out(path);
        out << R"({"robot_speed": 1.0, "future_key": "keep-me"})";
    }

    auto config = std::make_shared<des::SimConfig>();
    config->robotSpeed = 2.5;
    ASSERT_TRUE(ConfigLoader::saveSimConfig(path, config));

    std::ifstream in(path);
    nlohmann::json j;
    in >> j;
    EXPECT_EQ(j.at("future_key").get<std::string>(), "keep-me");
    EXPECT_DOUBLE_EQ(j.at("robot_speed").get<double>(), 2.5);
}

TEST(ConfigLoaderOrders, ExpandsRepeatingOrdersOverSimWindow) {
    const auto orders = ConfigLoader::loadOrderConfig(fixturesDir() + "/test_repeating_orders.json", 25200, 30 * 86400);
    ASSERT_TRUE(orders.has_value());
    ASSERT_EQ(orders->size(), 6u);

    std::vector<int> repeatedDeadlines;
    for (const auto& o : *orders) {
        if (o->id >= 200000) {
            repeatedDeadlines.push_back(*o->deadline);
        }
    }
    ASSERT_EQ(repeatedDeadlines.size(), 5u);
    EXPECT_EQ(repeatedDeadlines[0], 36000);
    EXPECT_EQ(repeatedDeadlines[1], 7 * 86400 + 36000);
    EXPECT_EQ(repeatedDeadlines[4], 28 * 86400 + 36000);

    const auto absolute = std::find_if(orders->begin(), orders->end(), [](const auto& o) { return o->id == 2; });
    ASSERT_NE(absolute, orders->end());
    EXPECT_EQ(*(*absolute)->deadline, 122400);
}

TEST(ConfigLoaderOrders, RepeatingOrderDefaultsToSingleDayWindow) {
    const auto orders = ConfigLoader::loadOrderConfig(fixturesDir() + "/test_repeating_orders.json");
    ASSERT_TRUE(orders.has_value());
    ASSERT_EQ(orders->size(), 2u);
}
