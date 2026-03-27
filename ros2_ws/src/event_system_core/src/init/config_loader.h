#pragma once
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <vector>

#include "../util/types.h"

using AppointmentList = std::vector<std::shared_ptr<des::Appointment>>;

const std::string DEFAULT_APPOINTMENT_FILE = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/appointments.json";
const std::string DEFAULT_EMPLOYEE_FILE =  "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/employee.json";
const std::string SIM_CONFIG_FILE = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/sim_config.json";

class ConfigLoader {
public:
    static std::optional<AppointmentList> loadAppointmentConfig(const std::string& filePath) {
        auto json = getJson(filePath);
        if (!json.has_value()) {
            std::cout << "Use default appointment config file: " << DEFAULT_APPOINTMENT_FILE << std::endl;
            json = getJson(DEFAULT_APPOINTMENT_FILE);
            assert(json.has_value());
        }

        AppointmentList appointments;
        try {
            auto jAppts = json.value()["appointments"];
            appointments.reserve(jAppts.size());

            int idCounter = 0;
            for (const auto& item : jAppts) {
                des::Appointment appointment;
                appointment.id = idCounter++;
                appointment.personName      = item.at("personName").get<std::string>();
                appointment.description     = item.at("description").get<std::string>();
                appointment.roomName        = item.at("roomName").get<std::string>();
                appointment.appointmentTime = item.at("appointmentTime").get<int>();

                appointments.emplace_back(std::make_shared<des::Appointment>(appointment));
            }

        } catch (const nlohmann::json::type_error& e) {
            std::cerr << "Failed to parse json file: " << filePath << std::endl;
            return std::nullopt;
        }
        return appointments;
    };

    static std::optional<std::vector<std::shared_ptr<des::Person>>> loadEmployees(const std::string& filePath = DEFAULT_EMPLOYEE_FILE) {
        auto jsonOpt = getJson(filePath);
        if (!jsonOpt.has_value()) {
            return std::nullopt;
        }

        std::vector<std::shared_ptr<des::Person>> employees;
        try {
            const auto& jsonArray = jsonOpt.value().at("employees");

            for (const auto& item : jsonArray) {
                des::Person p;
                p.id               = item.at("id").get<int>();
                p.firstName        = item.at("firstName").get<std::string>();
                p.lastName         = item.at("lastName").get<std::string>();
                p.birthDate        = item.at("birthDate").get<std::string>();
                p.sex              = item.at("sex").get<std::string>();
                p.workplace        = item.at("workplace").get<std::string>();
                p.color            = item.value("color", "");
                p.currentRoom      = item.at("currentRoom").get<std::string>();
                p.roomLabels       = item.at("roomLabels").get<std::vector<std::string>>();
                p.transitionMatrix = item.at("transitionMatrix").get<std::vector<std::vector<double>>>();

                if (p.transitionMatrix.size() != p.roomLabels.size()) {
                    std::cerr << "Warnung: Matrix-Dimension passt nicht zu roomLabels für " << p.firstName << std::endl;
                }

                employees.push_back(std::make_shared<des::Person>(p));
            }
        } catch (const nlohmann::json::exception& e) {
            std::cerr << "JSON Parsing Error: " << e.what() << std::endl;
            return std::nullopt;
        }

        return employees;
    }

    static std::optional<des::SimConfig> loadSimConfig(const std::string& filePath = SIM_CONFIG_FILE) {
        const auto json = getJson(filePath);
        if (!json.has_value()) {
            return std::nullopt;
        }

        try {
            auto j = json.value();
            des::SimConfig config;
            config.personFindProbability    = j.at("find_person_probability").get<double>();
            config.driveTimeStd             = j.at("drive_time_std").get<double>();
            config.robotSpeed               = j.at("robot_speed").get<double>();
            config.robotAccompanySpeed      = j.at("robot_accompany_speed").get<double>();
            config.conversationProbability  = j.at("conversation_probability").get<double>();
            config.conversationDurationStd  = j.at("conversation_duration_std").get<double>();
            config.conversationDurationMean = j.at("conversation_duration_mean").get<double>();
            config.timeBuffer               = j.at("timeBuffer").get<double>();
            config.energyConsumptionDrive   = j.at("energy_consumption_drive").get<double>();
            config.energyConsumptionBase    = j.at("energy_consumption_base").get<double>();
            config.batteryCapacity          = j.at("battery_capacity").get<double>();
            config.initialBatteryCapacity   = j.at("initial_battery_capacity").get<double>();
            config.chargingRate             = j.at("charging_rate").get<double>();
            config.lowBatteryThreshold      = j.at("low_battery_threshold").get<double>();
            config.fullBatteryThreshold     = j.at("full_battery_threshold").get<double>();
            config.arrivalMean              = j.value("arrival_mean", 9.0 * 3600);
            config.arrivalStd               = j.value("arrival_std", 3600.0);
            config.departureMean            = j.value("departure_mean", 17.0 * 3600);
            config.departureStd             = j.value("departure_std", 3600.0);
            config.arrivalDistribution      = des::distributionTypeFromString(j.value("arrival_distribution", "normal"));
            config.departureDistribution    = des::distributionTypeFromString(j.value("departure_distribution", "normal"));
            config.dockLocation             = j.at("dock_location").get<std::string>();
            config.cacheEnabled             = j.at("cacheEnabled").get<bool>();

            if (j.contains("appointments_path")) {
                config.appointmentsPath = j.at("appointments_path").get<std::string>();
            } else {
                config.appointmentsPath = "appointments.json";
            }
            return config;
        } catch (const nlohmann::json::type_error& e) {
            std::cerr << "Failed to parse sim config json: " << filePath << std::endl;
            return std::nullopt;
        }
    }

    static std::vector<std::shared_ptr<des::Person>> filterByAppointments(
        const std::vector<std::shared_ptr<des::Person>>& employees,
        const AppointmentList& appointments
    ) {
        std::set<std::string> needed;
        for (const auto& appt : appointments) {
            needed.insert(appt->personName);
        }
        std::vector<std::shared_ptr<des::Person>> filtered;
        for (const auto& p : employees) {
            if (needed.contains(p->firstName)) {
                filtered.push_back(p);
            }
        }
        return filtered;
    }

    static void validateConfig(
        const AppointmentList& appointments,
        const std::vector<std::shared_ptr<des::Person>>& employees,
        const std::map<std::string, des::Point>& locationMap,
        const std::string& arrivalLocation
    ) {
        std::vector<std::string> errors;

        // Build employee lookup
        std::set<std::string> employeeNames;
        for (const auto& p : employees) {
            employeeNames.insert(p->firstName);
        }

        // Check appointments reference valid employees and rooms
        for (const auto& appt : appointments) {
            if (!employeeNames.contains(appt->personName)) {
                errors.push_back("Appointment '" + appt->description +
                    "': personName '" + appt->personName + "' does not match any employee");
            }
            if (!locationMap.contains(appt->roomName)) {
                errors.push_back("Appointment '" + appt->description +
                    "': roomName '" + appt->roomName + "' is not a known location");
            }
        }

        // Check employee roomLabels reference valid rooms and arrival location is reachable
        for (const auto& p : employees) {
            for (const auto& room : p->roomLabels) {
                if (!locationMap.contains(room)) {
                    errors.push_back("Employee '" + p->firstName +
                        "': roomLabel '" + room + "' is not a known location");
                }
            }

            if (!arrivalLocation.empty()) {
                auto it = std::find(p->roomLabels.begin(), p->roomLabels.end(), arrivalLocation);
                if (it == p->roomLabels.end()) {
                    errors.push_back("Employee '" + p->firstName +
                        "': arrivalLocation '" + arrivalLocation + "' is not in roomLabels");
                }
            }

            // Check transition matrix dimensions
            if (p->transitionMatrix.size() != p->roomLabels.size()) {
                errors.push_back("Employee '" + p->firstName +
                    "': transitionMatrix has " + std::to_string(p->transitionMatrix.size()) +
                    " rows but roomLabels has " + std::to_string(p->roomLabels.size()) + " entries");
            }
            for (size_t i = 0; i < p->transitionMatrix.size(); ++i) {
                if (p->transitionMatrix[i].size() != p->roomLabels.size()) {
                    errors.push_back("Employee '" + p->firstName +
                        "': transitionMatrix row " + std::to_string(i) +
                        " has " + std::to_string(p->transitionMatrix[i].size()) +
                        " columns but expected " + std::to_string(p->roomLabels.size()));
                }
            }
        }

        if (!errors.empty()) {
            std::string msg = "Configuration validation failed:\n";
            for (const auto& e : errors) {
                msg += "  - " + e + "\n";
            }
            throw std::runtime_error(msg);
        }
    }

    static bool saveSimConfig(std::string filePath, std::shared_ptr<des::SimConfig> config) {
        nlohmann::json j;
        j["find_person_probability"] = config->personFindProbability;
        j["drive_time_std"] = config->driveTimeStd;
        j["robot_speed"] = config->robotSpeed;
        j["robot_accompany_speed"] = config->robotAccompanySpeed;
        j["conversation_probability"] = config->conversationProbability;
        j["conversation_duration_std"] = config->conversationDurationStd;
        j["conversation_duration_mean"] = config->conversationDurationMean;
        j["timeBuffer"] = config->timeBuffer;
        j["energy_consumption_drive"] = config->energyConsumptionDrive;
        j["energy_consumption_base"] = config->energyConsumptionBase;
        j["battery_capacity"] = config->batteryCapacity;
        j["initial_battery_capacity"] = config->initialBatteryCapacity;
        j["charging_rate"] = config->chargingRate;
        j["low_battery_threshold"] = config->lowBatteryThreshold;
        j["full_battery_threshold"] = config->fullBatteryThreshold;
        j["arrival_mean"] = config->arrivalMean;
        j["arrival_std"] = config->arrivalStd;
        j["departure_mean"] = config->departureMean;
        j["departure_std"] = config->departureStd;
        j["arrival_distribution"] = des::distributionTypeToString(config->arrivalDistribution);
        j["departure_distribution"] = des::distributionTypeToString(config->departureDistribution);
        j["dock_location"] = config->dockLocation;
        j["cacheEnabled"] = config->cacheEnabled;
        j["appointments_path"] = config->appointmentsPath;

        std::ofstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Could not write to file: " << filePath << std::endl;
            return false;
        }
        file << std::setw(4) << j << std::endl;
        return true;
    }

private:
    static std::optional<nlohmann::json> getJson(const std::string& filePath) {
        // open file
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Could not read file from path: " << filePath << std::endl;
            return std::nullopt;
        }

        // read file
        nlohmann::json json;
        try {
            file >> json;
        } catch (const nlohmann::json::parse_error& e) {
            std::cerr << "Could not parse json file: " << filePath << std::endl;
            return std::nullopt;
        }
        return json;
    }
};
