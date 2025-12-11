#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <vector>
#include <optional>

#include "../util/types.h"


using AppointmentList = std::vector<std::shared_ptr<des::Appointment>>;

class ConfigLoader {
public:
    static std::optional<std::shared_ptr<des::SimConfig>> loadDESConfig(std::string filePath) {
        auto jsonOpt = getJson(filePath);
        if(!jsonOpt.has_value()) {
            return std::nullopt;
        }

        auto json = jsonOpt.value();
        
        des::SimConfig config;
        try {
            config.personFindProbability   = json.at("find_person_probability");
            config.robotSpeed              = json.at("robot_speed");
            config.robotEscortSpeed        = json.at("robot_escort_speed");
            config.driveStd                = json.at("drive_std");
            config.conversationFoundStd    = json.at("conversation_found_std");
            config.conversationDropOffStd  = json.at("conversation_drop_off_std");
            config.missionOverhead         = json.at("missionOverhead");
            config.timeBuffer              = json.at("timeBuffer");


        } catch (const nlohmann::json::type_error& e) {
            std::cerr << "Failed to parse json file: " << filePath << std::endl;
            return std::nullopt;
        }
        return std::make_shared<des::SimConfig>(config);
    };

    static std::optional<AppointmentList>loadAppointmentConfig(
        std::string filePath, 
        int sim_start_time
    ) {
        auto json = getJson(filePath);
        if(!json.has_value()) {
            return std::nullopt;
        }

        AppointmentList appointments;
        try {
            auto jAppts= json.value()["appointments"];
            appointments.reserve(jAppts.size());

            int idCounter = 0;
            for (const auto& item : jAppts) {
                des::Appointment appointment;
                appointment.id = idCounter++;
                
                appointment.personName  = item.at("personName").get<std::string>();
                appointment.description = item.at("description").get<std::string>();
                appointment.roomName    = item.at("roomName").get<std::string>();
                
                int time = item.at("appointmentTime").get<int>();
                appointment.appointmentTime = time + sim_start_time;
            
                appointments.emplace_back(std::make_shared<des::Appointment>(appointment));
            }

        } catch (const nlohmann::json::type_error& e){
            std::cerr << "Failed to parse json file: " << filePath << std::endl;
            return std::nullopt;
        }
        return appointments;
    };


    static void printDESConfig(
        const std::shared_ptr<des::SimConfig> config, 
        const std::string& simFilePath, 
        const std::string& apptFilePath 
    ) {
        const int W = 25;

        std::cout << "\n" << "\033[1m" << "--- Configuration Loaded ---" << "\033[0m" << std::endl;

        std::cout << std::left << std::setw(W) << "Sim Config"         << ": " << simFilePath << std::endl;
        std::cout << std::left << std::setw(W) << "Apptiontemt Config" << ": " << apptFilePath<< std::endl;
        std::cout << std::left << std::setw(W) << "Person Find Prob."  << ": " << config.get()->personFindProbability << std::endl;
        std::cout << std::left << std::setw(W) << "Sim Speed"          << ": " << config.get()->robotSpeed << std::endl;
        std::cout << std::left << std::setw(W) << "Escort Speed"       << ": " << config.get()->robotEscortSpeed << std::endl;
        std::cout << std::left << std::setw(W) << "conversation_found_std"    << ": " << config.get()->conversationFoundStd<< std::endl;
        std::cout << std::left << std::setw(W) << "conversation_drop_off_std" << ": " << config.get()->conversationDropOffStd<< std::endl;

        std::cout << "----------------------------\n" << std::endl;
    }

private:
    static std::optional<nlohmann::json> getJson(std::string& filePath) {
        // open file
        std::ifstream file(filePath);
        if(!file.is_open()) {
            std::cerr << "Could not read file from path: " << filePath << std::endl;
            return std::nullopt;
        }

        // read file
        nlohmann::json json;
        try {
            file >> json;
        } catch (const nlohmann::json::parse_error& e){
            std::cerr << "Could not parse json file: " << filePath << std::endl;
            return std::nullopt;
        }
        return json;
    }
};
