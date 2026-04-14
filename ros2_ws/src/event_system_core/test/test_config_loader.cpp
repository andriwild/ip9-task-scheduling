#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <memory>

#include "../src/init/config_loader.h"
#include "../src/plugins/accompany/accompany_order.h"
#include "../src/plugins/accompany/accompany_plugin.h"
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

class PluginRegistry : public ::testing::Environment {
public:
    void SetUp() override {
        static bool registered = false;
        if (!registered) {
            OrderRegistry::instance().registerPlugin(std::make_unique<AccompanyOrderPlugin>());
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
    EXPECT_DOUBLE_EQ(result->robotAccompanySpeed, 0.3);
    EXPECT_DOUBLE_EQ(result->timeBuffer, 60.0);
    EXPECT_DOUBLE_EQ(result->batteryCapacity, 100.0);
    EXPECT_DOUBLE_EQ(result->initialBatteryCapacity, 80.0);
    EXPECT_DOUBLE_EQ(result->chargingRate, 0.5);
    EXPECT_DOUBLE_EQ(result->lowBatteryThreshold, 20.0);
    EXPECT_DOUBLE_EQ(result->fullBatteryThreshold, 95.0);
    EXPECT_EQ(result->dockLocation, "IMVS_Dock");
    EXPECT_FALSE(result->cacheEnabled);
    EXPECT_EQ(result->appointmentsPath, "test.json");
    EXPECT_DOUBLE_EQ(result->appointmentDuration, 1800.0);
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

// --- saveSimConfig roundtrip ---

TEST(ConfigLoaderSimConfig, SaveAndReloadProducesSameConfig) {
    auto original = ConfigLoader::loadSimConfig(fixturesDir() + "/test_sim_config.json");
    ASSERT_TRUE(original.has_value());

    std::string tmpFile = "/tmp/test_sim_config_roundtrip.json";
    auto configPtr = std::make_shared<des::SimConfig>(*original);
    ASSERT_TRUE(ConfigLoader::saveSimConfig(tmpFile, configPtr));

    auto reloaded = ConfigLoader::loadSimConfig(tmpFile);
    ASSERT_TRUE(reloaded.has_value());

    EXPECT_DOUBLE_EQ(original->robotSpeed, reloaded->robotSpeed);
    EXPECT_DOUBLE_EQ(original->robotAccompanySpeed, reloaded->robotAccompanySpeed);
    EXPECT_DOUBLE_EQ(original->timeBuffer, reloaded->timeBuffer);
    EXPECT_DOUBLE_EQ(original->batteryCapacity, reloaded->batteryCapacity);
    EXPECT_DOUBLE_EQ(original->initialBatteryCapacity, reloaded->initialBatteryCapacity);
    EXPECT_DOUBLE_EQ(original->chargingRate, reloaded->chargingRate);
    EXPECT_DOUBLE_EQ(original->lowBatteryThreshold, reloaded->lowBatteryThreshold);
    EXPECT_DOUBLE_EQ(original->fullBatteryThreshold, reloaded->fullBatteryThreshold);
    EXPECT_DOUBLE_EQ(original->energyConsumptionDrive, reloaded->energyConsumptionDrive);
    EXPECT_DOUBLE_EQ(original->energyConsumptionBase, reloaded->energyConsumptionBase);
    EXPECT_DOUBLE_EQ(original->conversationProbability, reloaded->conversationProbability);
    EXPECT_DOUBLE_EQ(original->conversationDurationMean, reloaded->conversationDurationMean);
    EXPECT_DOUBLE_EQ(original->conversationDurationStd, reloaded->conversationDurationStd);
    EXPECT_DOUBLE_EQ(original->driveTimeStd, reloaded->driveTimeStd);
    EXPECT_EQ(original->dockLocation, reloaded->dockLocation);
    EXPECT_EQ(original->cacheEnabled, reloaded->cacheEnabled);
    EXPECT_EQ(original->appointmentsPath, reloaded->appointmentsPath);
    EXPECT_EQ(original->arrivalDistribution, reloaded->arrivalDistribution);
    EXPECT_EQ(original->departureDistribution, reloaded->departureDistribution);
    EXPECT_DOUBLE_EQ(original->appointmentDuration, reloaded->appointmentDuration);

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

    std::map<std::string, des::Point> locationMap;
    locationMap["5.2B03"] = des::Point(0, 0, 0);
    locationMap["5.2B01"] = des::Point(1, 1, 0);
    locationMap["5.2B10"] = des::Point(2, 2, 0);
    locationMap["5.2B_Elevator"] = des::Point(3, 3, 0);

    EXPECT_NO_THROW(ConfigLoader::validateConfig(*orders, *employees, locationMap, "5.2B_Elevator"));
}

TEST(ConfigLoaderValidation, UnknownPersonInAccompanyOrderThrows) {
    des::PersonList employees;
    auto emp = std::make_shared<des::Person>();
    emp->firstName = "Max";
    emp->roomLabels = {"RoomA"};
    emp->transitionMatrix = {{1.0}};
    employees.push_back(emp);

    des::OrderList orders = { makeAccompanyOrder("UnknownPerson", "RoomA") };

    std::map<std::string, des::Point> locationMap;
    locationMap["RoomA"] = des::Point(0, 0, 0);

    EXPECT_THROW(ConfigLoader::validateConfig(orders, employees, locationMap, "RoomA"), std::runtime_error);
}

TEST(ConfigLoaderValidation, UnknownRoomInAccompanyOrderThrows) {
    auto emp = std::make_shared<des::Person>();
    emp->firstName = "Max";
    emp->roomLabels = {"RoomA"};
    emp->transitionMatrix = {{1.0}};

    des::OrderList orders = { makeAccompanyOrder("Max", "NonexistentRoom") };

    std::map<std::string, des::Point> locationMap;
    locationMap["RoomA"] = des::Point(0, 0, 0);

    EXPECT_THROW(
        ConfigLoader::validateConfig(orders, {emp}, locationMap, "RoomA"),
        std::runtime_error
    );
}

TEST(ConfigLoaderValidation, MismatchedTransitionMatrixThrows) {
    auto emp = std::make_shared<des::Person>();
    emp->firstName = "Max";
    emp->roomLabels = {"RoomA", "RoomB"};
    emp->transitionMatrix = {{1.0}}; // 1x1 but should be 2x2

    std::map<std::string, des::Point> locationMap;
    locationMap["RoomA"] = des::Point(0, 0, 0);
    locationMap["RoomB"] = des::Point(1, 1, 0);

    EXPECT_THROW(
        ConfigLoader::validateConfig({}, {emp}, locationMap, "RoomA"),
        std::runtime_error
    );
}
