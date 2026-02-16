#pragma once
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

#include "../util/types.h"

using AppointmentList = std::vector<std::shared_ptr<des::Appointment>>;

class ConfigLoader {
public:
    static std::optional<AppointmentList> loadAppointmentConfig(std::string filePath) {
        auto json = getJson(filePath);
        if (!json.has_value()) {
            return std::nullopt;
        }

        AppointmentList appointments;
        try {
            auto jAppts = json.value()["appointments"];
            appointments.reserve(jAppts.size());

            int idCounter = 0;
            for (const auto& item : jAppts) {
                des::Appointment appointment;
                appointment.id = idCounter++;
                appointment.personName = item.at("personName").get<std::string>();
                appointment.description = item.at("description").get<std::string>();
                appointment.roomName = item.at("roomName").get<std::string>();
                appointment.appointmentTime = item.at("appointmentTime").get<int>();

                appointments.emplace_back(std::make_shared<des::Appointment>(appointment));
            }

        } catch (const nlohmann::json::type_error& e) {
            std::cerr << "Failed to parse json file: " << filePath << std::endl;
            return std::nullopt;
        }
        return appointments;
    };

    static std::optional<std::map<std::string, std::vector<std::string>>> loadEmployeeLocations(std::string filePath) {
        auto json = getJson(filePath);
        if (!json.has_value()) {
            return std::nullopt;
        }

        std::map<std::string, std::vector<std::string>> employees;
        try {
            for (const auto& item : json.value()["employees"]) {
                auto name = item.at("name").get<std::string>();
                auto locations = item.at("locations").get<std::vector<std::string>>();
                employees[name] = locations;
            }
        } catch (const nlohmann::json::type_error& e) {
            std::cerr << "Failed to parse employee locations json: " << filePath << std::endl;
            return std::nullopt;
        }
        return employees;
    }

    static std::optional<des::SimConfig> loadSimConfig(std::string filePath) {
        const auto json = getJson(filePath);
        if (!json.has_value()) {
            return std::nullopt;
        }

        try {
            auto j = json.value();
            des::SimConfig config;
            config.personFindProbability = j.at("find_person_probability").get<double>();
            config.driveTimeStd = j.at("drive_time_std").get<double>();
            config.robotSpeed = j.at("robot_speed").get<double>();
            config.robotAccompanySpeed = j.at("robot_accompany_speed").get<double>();
            config.conversationProbability = j.at("conversation_probability").get<double>();
            config.conversationDurationStd = j.at("conversation_duration_std").get<double>();
            config.conversationDurationMean = j.at("conversation_duration_mean").get<double>();
            config.timeBuffer = j.at("timeBuffer").get<double>();
            config.energyConsumptionDrive = j.at("energy_consumption_drive").get<double>();
            config.energyConsumptionBase = j.at("energy_consumption_base").get<double>();
            config.batteryCapacity = j.at("battery_capacity").get<double>();
            config.initialBatteryCapacity = j.at("initial_battery_capacity").get<double>();
            config.chargingRate = j.at("charging_rate").get<double>();
            config.lowBatteryThreshold = j.at("low_battery_threshold").get<double>();
            config.dockPose = j.at("dock_pose").get<std::vector<double>>();
            config.cacheEnabled = j.at("cacheEnabled").get<bool>();

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
        j["dock_pose"] = config->dockPose;
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
