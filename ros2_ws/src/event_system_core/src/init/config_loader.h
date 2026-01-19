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
    static std::optional<AppointmentList> loadAppointmentConfig(std::string filePath, int sim_start_time) {
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

                int time = item.at("appointmentTime").get<int>();
                appointment.appointmentTime = time + sim_start_time;

                appointments.emplace_back(std::make_shared<des::Appointment>(appointment));
            }

        } catch (const nlohmann::json::type_error& e) {
            std::cerr << "Failed to parse json file: " << filePath << std::endl;
            return std::nullopt;
        }
        return appointments;
    };

    static std::optional<des::SimConfig> loadSimConfig(std::string filePath) {
        auto json = getJson(filePath);
        if (!json.has_value()) {
            return std::nullopt;
        }

        try {
            auto j = json.value();
            des::SimConfig config;
            config.personFindProbability    = j.at("find_person_probability").get<double>();
            config.driveStd                 = j.at("drive_std").get<double>();
            config.robotSpeed               = j.at("robot_speed").get<double>();
            config.robotAccompanySpeed         = j.at("robot_accompany_speed").get<double>();
            config.conversationFoundStd     = j.at("conversation_found_std").get<double>();
            config.conversationDropOffStd   = j.at("conversation_drop_off_std").get<double>();
            config.missionOverhead          = j.at("missionOverhead").get<double>();
            config.timeBuffer               = j.at("timeBuffer").get<double>();

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
        j["find_person_probability"]    = config->personFindProbability;
        j["drive_std"]                  = config->driveStd;
        j["robot_speed"]                = config->robotSpeed;
        j["robot_accompany_speed"]      = config->robotAccompanySpeed;
        j["conversation_found_std"]     = config->conversationFoundStd;
        j["conversation_drop_off_std"]  = config->conversationDropOffStd;
        j["missionOverhead"]            = config->missionOverhead;
        j["timeBuffer"]                 = config->timeBuffer;
        j["appointments_path"]          = config->appointmentsPath;

        std::ofstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Could not write to file: " << filePath << std::endl;
            return false;
        }
        file << std::setw(4) << j << std::endl;
        return true;
    }


private:
    static std::optional<nlohmann::json> getJson(std::string& filePath) {
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
