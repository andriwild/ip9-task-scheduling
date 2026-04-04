#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <memory>

#include "../src/init/config_loader.h"

namespace {

std::string fixturesDir() {
    return std::string(TEST_FIXTURES_DIR);
}

} // namespace

// --- loadAppointmentConfig ---

TEST(ConfigLoaderAppointments, LoadsValidAppointments) {
    auto result = ConfigLoader::loadAppointmentConfig(fixturesDir() + "/test_appointments.json");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2u);

    EXPECT_EQ((*result)[0]->personName, "Max");
    EXPECT_EQ((*result)[0]->roomName, "5.2B10");
    EXPECT_EQ((*result)[0]->appointmentTime, 36000);
    EXPECT_EQ((*result)[0]->description, "Meeting A");
    EXPECT_EQ((*result)[0]->id, 0);

    EXPECT_EQ((*result)[1]->personName, "Anna");
    EXPECT_EQ((*result)[1]->id, 1);
}

TEST(ConfigLoaderAppointments, NonexistentFileFallsBackToDefault) {
    auto result = ConfigLoader::loadAppointmentConfig("/tmp/nonexistent_xyz_12345.json");
    // Function falls back to DEFAULT_APPOINTMENT_FILE - verify it loads something
    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(result->empty());
    // Verify appointments have required fields populated
    for (const auto& appt : *result) {
        EXPECT_FALSE(appt->personName.empty());
        EXPECT_FALSE(appt->roomName.empty());
        EXPECT_GT(appt->appointmentTime, 0);
    }
}

TEST(ConfigLoaderAppointments, InvalidJsonStructureReturnsNullopt) {
    auto result = ConfigLoader::loadAppointmentConfig(fixturesDir() + "/invalid_appointments.json");
    EXPECT_FALSE(result.has_value());
}

TEST(ConfigLoaderAppointments, AppointmentIdsAreSequential) {
    auto result = ConfigLoader::loadAppointmentConfig(fixturesDir() + "/test_appointments.json");
    ASSERT_TRUE(result.has_value());
    for (size_t i = 0; i < result->size(); ++i) {
        EXPECT_EQ((*result)[i]->id, static_cast<int>(i));
    }
}

TEST(ConfigLoaderAppointments, DefaultStateIsPending) {
    auto result = ConfigLoader::loadAppointmentConfig(fixturesDir() + "/test_appointments.json");
    ASSERT_TRUE(result.has_value());
    for (const auto& appt : *result) {
        EXPECT_EQ(appt->state, des::MissionState::PENDING);
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

    EXPECT_DOUBLE_EQ(result->personFindProbability, 0.8);
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

    EXPECT_DOUBLE_EQ(original->personFindProbability, reloaded->personFindProbability);
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

    std::filesystem::remove(tmpFile);
}

TEST(ConfigLoaderSimConfig, SaveToInvalidPathReturnsFalse) {
    auto config = std::make_shared<des::SimConfig>();
    EXPECT_FALSE(ConfigLoader::saveSimConfig("/nonexistent_dir/file.json", config));
}

// --- filterByAppointments ---

TEST(ConfigLoaderFilter, FiltersEmployeesByAppointment) {
    auto employees = ConfigLoader::loadEmployees(fixturesDir() + "/test_employees.json");
    ASSERT_TRUE(employees.has_value());

    // Only one appointment for Max
    AppointmentList appointments;
    auto appt = std::make_shared<des::Appointment>();
    appt->personName = "Max";
    appt->roomName = "5.2B10";
    appt->appointmentTime = 36000;
    appointments.push_back(appt);

    auto filtered = ConfigLoader::filterByAppointments(*employees, appointments);
    ASSERT_EQ(filtered.size(), 1u);
    EXPECT_EQ(filtered[0]->firstName, "Max");
}

TEST(ConfigLoaderFilter, NoMatchingEmployeesReturnsEmpty) {
    auto employees = ConfigLoader::loadEmployees(fixturesDir() + "/test_employees.json");
    ASSERT_TRUE(employees.has_value());

    AppointmentList appointments;
    auto appt = std::make_shared<des::Appointment>();
    appt->personName = "UnknownPerson";
    appointments.push_back(appt);

    auto filtered = ConfigLoader::filterByAppointments(*employees, appointments);
    EXPECT_TRUE(filtered.empty());
}

TEST(ConfigLoaderFilter, AllEmployeesMatchedWhenAllHaveAppointments) {
    auto employees = ConfigLoader::loadEmployees(fixturesDir() + "/test_employees.json");
    ASSERT_TRUE(employees.has_value());

    AppointmentList appointments;
    for (const auto& emp : *employees) {
        auto appt = std::make_shared<des::Appointment>();
        appt->personName = emp->firstName;
        appointments.push_back(appt);
    }

    auto filtered = ConfigLoader::filterByAppointments(*employees, appointments);
    EXPECT_EQ(filtered.size(), employees->size());
}

// --- validateConfig ---

TEST(ConfigLoaderValidation, ValidConfigDoesNotThrow) {
    auto employees = ConfigLoader::loadEmployees(fixturesDir() + "/test_employees.json");
    auto appointments = ConfigLoader::loadAppointmentConfig(fixturesDir() + "/test_appointments.json");
    ASSERT_TRUE(employees.has_value());
    ASSERT_TRUE(appointments.has_value());

    std::map<std::string, des::Point> locationMap;
    locationMap["5.2B03"] = des::Point(0, 0, 0);
    locationMap["5.2B01"] = des::Point(1, 1, 0);
    locationMap["5.2B10"] = des::Point(2, 2, 0);
    locationMap["5.2B_Elevator"] = des::Point(3, 3, 0);

    EXPECT_NO_THROW(ConfigLoader::validateConfig(*appointments, *employees, locationMap, "5.2B_Elevator"));
}

TEST(ConfigLoaderValidation, UnknownPersonInAppointmentThrows) {
    std::vector<std::shared_ptr<des::Person>> employees;
    auto emp = std::make_shared<des::Person>();
    emp->firstName = "Max";
    emp->roomLabels = {"RoomA"};
    emp->transitionMatrix = {{1.0}};
    employees.push_back(emp);

    AppointmentList appointments;
    auto appt = std::make_shared<des::Appointment>();
    appt->personName = "UnknownPerson";
    appt->roomName = "RoomA";
    appt->description = "Test";
    appointments.push_back(appt);

    std::map<std::string, des::Point> locationMap;
    locationMap["RoomA"] = des::Point(0, 0, 0);

    EXPECT_THROW(ConfigLoader::validateConfig(appointments, employees, locationMap, "RoomA"), std::runtime_error);
}

TEST(ConfigLoaderValidation, UnknownRoomInAppointmentThrows) {
    auto emp = std::make_shared<des::Person>();
    emp->firstName = "Max";
    emp->roomLabels = {"RoomA"};
    emp->transitionMatrix = {{1.0}};

    auto appt = std::make_shared<des::Appointment>();
    appt->personName = "Max";
    appt->roomName = "NonexistentRoom";
    appt->description = "Test";

    std::map<std::string, des::Point> locationMap;
    locationMap["RoomA"] = des::Point(0, 0, 0);

    EXPECT_THROW(
        ConfigLoader::validateConfig({appt}, {emp}, locationMap, "RoomA"),
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
