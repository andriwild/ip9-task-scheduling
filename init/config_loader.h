#include "../util/types.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <qevent.h>
#include <qjsondocument.h>
#include <qjsonobject.h>
#include <qobject.h>
#include <qregion.h>
#include <vector>
#include <optional>


struct RobotConfig {
    double personFindProbability;
    double robot_speed;
    double robot_escort_speed;
};


class ConfigLoader {
public:
    static std::optional<RobotConfig>loadRobotConfig(std::string filePath) {
        auto jsonOpt = getJson(filePath);
        if(!jsonOpt.has_value()) {
            return std::nullopt;
        }

        auto json = jsonOpt.value();
        
        // parse config
        RobotConfig config;
        try {
            config.personFindProbability = json.at("find_person_probability");
            config.robot_speed = json.at("robot_speed");
            config.robot_escort_speed = json.at("robot_escort_speed");
        } catch (const nlohmann::json::type_error& e){
            std::cerr << "Failed to parse json file: " << filePath << std::endl;
            return std::nullopt;
        }
        return config;
    };

    static std::optional<std::vector<des::Appointment>>loadAppointmentConfig(
        std::string filePath, 
        int sim_start_time
    ) {
        auto json = getJson(filePath);
        if(!json.has_value()) {
            return std::nullopt;
        }

        std::vector<des::Appointment> appointments;
        try {
            auto jAppts= json.value()["appointments"];
            appointments.reserve(jAppts.size());

            int idCounter = 0;
            for (const auto& item : jAppts) {
                des::Appointment appointment;
                appointment.id = idCounter++;
                
                appointment.personName      = item.at("personName").get<std::string>();
                appointment.description     = item.at("description").get<std::string>();
                appointment.roomName        = item.at("roomName").get<std::string>();
                
                int time = item.at("appointmentTime").get<int>();
                appointment.appointmentTime = time + sim_start_time;
            
                appointments.emplace_back(appointment);
            }

        } catch (const nlohmann::json::type_error& e){
            std::cerr << "Failed to parse json file: " << filePath << std::endl;
            return std::nullopt;
        }
        return appointments;
    };


    static void printRobotConfig(
        const RobotConfig& config, 
        const std::string& robotFilePath, 
        const std::string& apptFilePath 
    ) {
        const int W = 25;

        std::cout << "\n" << "\033[1m" << "--- Configuration Loaded ---" << "\033[0m" << std::endl;

        std::cout << std::left << std::setw(W) << "Robot Config"         << ": " << robotFilePath << std::endl;
        std::cout << std::left << std::setw(W) << "Apptiontemt Config"   << ": " << apptFilePath<< std::endl;
        std::cout << std::left << std::setw(W) << "Person Find Prob."    << ": " << config.personFindProbability << std::endl;
        std::cout << std::left << std::setw(W) << "Robot Speed"          << ": " << config.robot_speed << std::endl;
        std::cout << std::left << std::setw(W) << "Escort Speed"         << ": " << config.robot_escort_speed << std::endl;

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
