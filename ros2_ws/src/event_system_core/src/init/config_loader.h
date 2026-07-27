#pragma once
#include <fstream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <sstream>
#include <vector>

#include "../util/log.h"
#include "../util/types.h"
#include "../plugins/order_registry.h"
#include "../plugins/accompany/accompany_order.h"


const std::string DEFAULT_ORDER_FILE    = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/appointments.json";
const std::string DEFAULT_EMPLOYEE_FILE = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/employee.json";
const std::string SIM_CONFIG_FILE       = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/sim_config.json";
const std::string BUILDING_FILE         = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/building.json";

constexpr int SIM_START_TIME = 25200;  // 07:00
constexpr int SIM_DURATION   = 43200;
constexpr int SECONDS_PER_DAY_CFG = 86400;

struct InterruptGeneratorConfig {
    std::string type;
    des::ExecutionMode execution;   // always INTERRUPT
    des::DistributionType distribution;
    double ratePerSecond;     // rate_per_hour / 3600
    int from;
    int to;
    nlohmann::json params;
};

struct BackgroundTemplate {
    nlohmann::json json;
    int everyNDays;
};

class ConfigLoader {
public:
    static inline std::string s_overridePath = "";

    static std::string configDir() {
        const auto slash = SIM_CONFIG_FILE.rfind('/');
        return slash == std::string::npos ? "" : SIM_CONFIG_FILE.substr(0, slash + 1);
    }

    static std::optional<des::OrderList> loadOrderConfig(const std::string& filePath, const int simStartTime = 0, const int simEndTime = SECONDS_PER_DAY_CFG) {

        auto json = getJson(filePath);
        if (!json.has_value()) {
            DES_LOG_INFO(rclcpp::get_logger("des.io.config"), "Use default appointment config file: %s", DEFAULT_ORDER_FILE.c_str());
            json = getJson(DEFAULT_ORDER_FILE);
            assert(json.has_value());
        }

        des::OrderList orders;
        int instanceId = 200000;
        for (const auto& j : json.value().at("orders")) {
            const std::string& type = j.at("type").get_ref<const std::string&>();
            auto& plugin = OrderRegistry::instance().get(type);

            const int everyNDays = j.value("every_n_days", 0);
            if (everyNDays <= 0) {
                orders.push_back(plugin.fromJson(j));
                continue;
            }

            const int offset = j.at("appointmentTime").get<int>();
            for (int day = 0; day * SECONDS_PER_DAY_CFG < simEndTime; day += everyNDays) {
                const int appointmentTime = day * SECONDS_PER_DAY_CFG + offset;
                if (appointmentTime < simStartTime || appointmentTime >= simEndTime) {
                    continue;
                }
                nlohmann::json instance = j;
                instance["id"] = instanceId++;
                instance["appointmentTime"] = appointmentTime;
                orders.push_back(plugin.fromJson(instance));
            }
        }
        return orders;
    };

    // Background orders are tasks without a fixed time scheduler dispatches them opportunistically
    // the "background" entries must not specify appointmentTime.
    static des::OrderList loadBackgroundOrders(const std::string& filePath) {
        auto json = getJson(filePath);
        if (!json.has_value() || !json.value().contains("background")) {
            return {};
        }

        des::OrderList orders;
        for (const auto& j : json.value().at("background")) {
            if (j.contains("appointmentTime")) {
                throw std::runtime_error(
                    "Background order (id=" + std::to_string(j.value("id", -1)) +
                    ") must not specify appointmentTime. Move it to 'orders' if it needs a fixed time.");
            }
            const std::string& type = j.at("type").get_ref<const std::string&>();
            auto order = OrderRegistry::instance().get(type).fromJson(j);
            order->execution = des::ExecutionMode::BACKGROUND;
            orders.push_back(order);
        }
        return orders;
    }

    static std::vector<BackgroundTemplate> loadBackgroundTemplates(const std::string& filePath) {
        auto json = getJson(filePath);
        if (!json.has_value() || !json.value().contains("background")) {
            return {};
        }

        std::vector<BackgroundTemplate> templates;
        for (const auto& j : json.value().at("background")) {
            if (j.contains("appointmentTime")) {
                throw std::runtime_error(
                    "Background order (id=" + std::to_string(j.value("id", -1)) + ") must not specify appointmentTime");
            }
            templates.push_back({ j, j.value("every_n_days", 0) });
        }
        return templates;
    }

    static std::optional<std::vector<InterruptGeneratorConfig>> loadInterruptGenerators(const std::string& filePath) {
        auto json = getJson(filePath);
        if (!json.has_value()) {
            DES_LOG_INFO(rclcpp::get_logger("des.io.config"), "No scenario file found at %s — skipping ad-hoc generators", filePath.c_str());
            return std::vector<InterruptGeneratorConfig>{};
        }
        if (!json.value().contains("ad_hoc_generators")) {
            return std::vector<InterruptGeneratorConfig>{};
        }

        std::vector<InterruptGeneratorConfig> generators;
        for (const auto& j : json.value().at("ad_hoc_generators")) {
            const std::string& type                  = j.at("type").get_ref<const std::string&>();
            const des::DistributionType distribution = des::distributionTypeFromString(j.value("distribution", "exponential"));
            const double ratePerHour                 = j.at("rate_per_hour").get<double>() / 3600.0;
            const int from                           = j.at("active_window").at("from").get<int>();
            const int to                             = j.at("active_window").at("to").get<int>();
            const nlohmann::json params              = j.at("params");

            if (j.contains("execution")) {
                const std::string& exec = j.at("execution").get_ref<const std::string&>();
                if (exec != "interrupt") {
                    throw std::runtime_error(
                        "Interrupt generator (type='" + type + "') has execution='" + exec);
                }
            }
            const des::ExecutionMode execution = des::ExecutionMode::INTERRUPT;

            generators.push_back({type, execution, distribution, ratePerHour, from, to, params });
        }
        return generators;
    };

    static std::optional<des::PersonList> loadEmployees(const std::string& filePath = DEFAULT_EMPLOYEE_FILE) {
        auto jsonOpt = getJson(filePath);
        if (!jsonOpt.has_value()) {
            return std::nullopt;
        }

        des::PersonList employees;
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
                p.roomLabels       = item.at("roomLabels").get<std::vector<std::string>>();
                p.transitionMatrix = item.at("transitionMatrix").get<std::vector<std::vector<double>>>();

                if (p.transitionMatrix.size() != p.roomLabels.size()) {
                    DES_LOG_WARN(rclcpp::get_logger("des.io.config"), "Matrix dimension does not match roomLabels for %s", p.firstName.c_str());
                }

                employees.push_back(std::make_unique<des::Person>(std::move(p)));
            }
        } catch (const nlohmann::json::exception& e) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.config"), "JSON Parsing Error: %s", e.what());
            return std::nullopt;
        }

        return employees;
    }

    static std::optional<des::SimConfig> loadSimConfig(const std::string& filePath = SIM_CONFIG_FILE, const std::string& overridePath = s_overridePath) {
        const auto json = getJson(filePath);
        if (!json.has_value()) {
            return std::nullopt;
        }

        try {
            auto j = json.value();

            if (!overridePath.empty()) {
                const std::string resolved = overridePath.front() == '/' ? overridePath : configDir() + overridePath;
                const auto overrideJson = getJson(resolved);
                if (!overrideJson.has_value()) {
                    DES_LOG_ERROR(rclcpp::get_logger("des.io.config"), "Could not read override config: %s", resolved.c_str());
                    return std::nullopt;
                }
                j.merge_patch(overrideJson.value());
                DES_LOG_INFO(rclcpp::get_logger("des.io.config"), "Applied config override: %s", resolved.c_str());
            }
            des::SimConfig config;
            config.driveTimeStd             = j.at("drive_time_std").get<double>();
            config.robotSpeed               = j.at("robot_speed").get<double>();
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
            config.lunchMean                = j.value("lunch_mean", 12.0 * 3600);
            config.lunchStd                 = j.value("lunch_std", 1800.0);
            config.lunchDistribution        = des::distributionTypeFromString(j.value("lunch_distribution", "normal"));
            config.lunchDurationMean        = j.value("lunch_duration_mean", 2400.0);
            config.lunchDurationStd         = j.value("lunch_duration_std", 600.0);
            config.simSpeedFactor           = j.value("sim_speed_factor", 0.0);
            config.dockLocation             = j.at("dock_location").get<std::string>();
            config.cacheEnabled             = j.at("cacheEnabled").get<bool>();

            if (j.contains("appointments_path")) {
                config.appointmentsPath = j.at("appointments_path").get<std::string>();
            } else {
                config.appointmentsPath = "appointments.json";
            }
            config.employeesPath = j.value("employees_path", DEFAULT_EMPLOYEE_FILE);
            config.peopleSpawnLocation = j.value("people_spawn_location", std::string("IMVS_Entrance"));
            config.personDetectionRange = j.value("person_detection_range", 5.0);
            config.personSpeed = j.value("person_speed", 1.4);
            config.simStartTime = j.value("sim_start_time", SIM_START_TIME);
            config.simDuration  = j.value("sim_duration",   SIM_DURATION);
            config.useDistanceMatrix = j.value("use_distance_matrix", false);
            config.batteryVoltage = j.value("battery_voltage", 12.0);
            config.cvThreshold    = j.value("cv_threshold", 0.8);
            config.taperFraction  = j.value("taper_fraction", 0.5);
            config.chargeToFull   = j.value("charge_to_full", true);
            config.alwaysChargeAtDock = j.value("always_charge_at_dock", false);
            config.metricsCsvExport   = j.value("metrics_csv_export", true);
            config.replanBackgroundOnInterrupt = j.value("replan_background_on_interrupt", true);
            config.searchExcludedRooms = j.value("search_excluded_rooms", std::vector<std::string>{"Elevator", "Stairwell", "Dock"});
            config.missionTraceExport = j.value("mission_trace_export", false);
            config.searchRewardStrategy = des::searchRewardStrategyFromString(j.value("search_reward_strategy", "beta_smoothed"));

            for (auto* plugin : OrderRegistry::instance().all()) {
                plugin->loadConfig(j.value(plugin->typeName(), nlohmann::json::object()));
            }
            return config;
        } catch (const nlohmann::json::type_error& e) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.config"), "Failed to parse sim config json: %s", filePath.c_str());
            return std::nullopt;
        }
    }

    static std::vector<des::Person*> filterByAppointments(
        const des::PersonList& employees,
        const des::OrderList& orders
    ) {
        std::set<std::string> needed;
        for (const auto& order : orders) {
            if (auto accompany = std::dynamic_pointer_cast<AccompanyOrder>(order)) {
                needed.insert(accompany->personName);
            }
        }
        std::vector<des::Person*> filtered;
        for (const auto& p : employees) {
            if (needed.contains(p->firstName)) {
                filtered.push_back(p.get());
            }
        }
        return filtered;
    }

    static void validateConfig(
        const des::OrderList& orders,
        const des::PersonList& employees,
        const des::LocationMap& locationMap,
        const std::string& arrivalLocation
    ) {
        std::vector<std::string> errors;

        // Build employee lookup
        std::set<std::string> employeeNames;
        for (const auto& p : employees) {
            employeeNames.insert(p->firstName);
        }

        // Check accompany orders reference valid employees and rooms.
        // Other order types provide their own validation elsewhere.
        for (const auto& order : orders) {
            auto accompany = std::dynamic_pointer_cast<AccompanyOrder>(order);
            if (!accompany) { continue; }
            if (!employeeNames.contains(accompany->personName)) {
                errors.push_back("Order '" + accompany->description +
                    "': personName '" + accompany->personName + "' does not match any employee");
            }
            if (!locationMap.contains(accompany->roomName)) {
                errors.push_back("Order '" + accompany->description +
                    "': roomName '" + accompany->roomName + "' is not a known location");
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
        nlohmann::json j = getJson(filePath).value_or(nlohmann::json::object());
        j["drive_time_std"] = config->driveTimeStd;
        j["robot_speed"] = config->robotSpeed;
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
        j["lunch_mean"] = config->lunchMean;
        j["lunch_std"] = config->lunchStd;
        j["lunch_distribution"] = des::distributionTypeToString(config->lunchDistribution);
        j["lunch_duration_mean"] = config->lunchDurationMean;
        j["lunch_duration_std"] = config->lunchDurationStd;
        j["sim_speed_factor"] = config->simSpeedFactor;
        j["dock_location"] = config->dockLocation;
        j["cacheEnabled"] = config->cacheEnabled;
        j["appointments_path"] = config->appointmentsPath;
        j["employees_path"] = config->employeesPath;
        j["people_spawn_location"] = config->peopleSpawnLocation;
        j["person_detection_range"] = config->personDetectionRange;
        j["person_speed"] = config->personSpeed;
        j["sim_start_time"] = config->simStartTime;
        j["sim_duration"]   = config->simDuration;
        j["use_distance_matrix"] = config->useDistanceMatrix;
        j["battery_voltage"] = config->batteryVoltage;
        j["cv_threshold"] = config->cvThreshold;
        j["taper_fraction"] = config->taperFraction;
        j["charge_to_full"] = config->chargeToFull;
        j["always_charge_at_dock"] = config->alwaysChargeAtDock;
        j["metrics_csv_export"] = config->metricsCsvExport;
        j["replan_background_on_interrupt"] = config->replanBackgroundOnInterrupt;
        j["search_excluded_rooms"] = config->searchExcludedRooms;
        j["mission_trace_export"] = config->missionTraceExport;
        j["search_reward_strategy"] = des::searchRewardStrategyToString(config->searchRewardStrategy);

        // each plugin serialises its own sub-object under
        for (auto* plugin : OrderRegistry::instance().all()) {
            auto sub = plugin->saveConfig();
            if (!sub.empty()) {
                j[plugin->typeName()] = std::move(sub);
            }
        }

        std::ofstream file(filePath);
        if (!file.is_open()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.config"), "Could not write to file: %s", filePath.c_str());
            return false;
        }
        file << std::setw(4) << j << std::endl;
        return true;
    }

    static std::optional<nlohmann::json> getJson(const std::string& filePath) {
        // open file
        std::ifstream file(filePath);
        if (!file.is_open()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.config"), "Could not read file from path: %s", filePath.c_str());
            return std::nullopt;
        }

        // read file
        nlohmann::json json;
        try {
            file >> json;
        } catch (const nlohmann::json::parse_error& e) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.config"), "Could not parse json file: %s", filePath.c_str());
            return std::nullopt;
        }
        return json;
    }

    // Loads the building snapshot into a name -> Location map (coords + optional area) 
    static std::string toursFilePath(double radius) {
        std::ostringstream oss;
        oss << "tours_r" << radius << ".json";
        const auto slash = BUILDING_FILE.rfind('/');
        const std::string dir = slash == std::string::npos ? "" : BUILDING_FILE.substr(0, slash + 1);
        return dir + oss.str();
    }

    static std::optional<des::RoomTourMap> loadRoomTours(const std::string& filePath) {
        const auto json = getJson(filePath);
        if (!json.has_value()) {
            return std::nullopt;
        }
        const auto& j = json.value();
        if (!j.contains("rooms")) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.config"), "Tour config %s missing 'rooms'", filePath.c_str());
            return std::nullopt;
        }

        des::RoomTourMap tours;
        for (const auto& [name, entry] : j.at("rooms").items()) {
            if (!entry.value("ok", false)) {
                continue;
            }
            des::RoomTour tour;
            tour.m_distance = entry.value("distance", 0.0);
            tour.m_steps = entry.value("steps", 0);
            if (entry.contains("path") && entry.at("path").is_array()) {
                for (const auto& p : entry.at("path")) {
                    tour.m_path.push_back(des::Point{p.at(0).get<double>(), p.at(1).get<double>(), 0.0});
                }
            }
            tours.emplace(name, tour);
        }
        return tours;
    }

    static std::optional<des::LocationMap> loadBuildingSnapshot(const std::string& filePath) {
        const auto json = getJson(filePath);
        if (!json.has_value()) {
            return std::nullopt;
        }
        const auto& j = json.value();
        if (!j.contains("names") || !j.contains("locations")) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.config"), "Building snapshot %s missing 'names'/'locations'", filePath.c_str());
            return std::nullopt;
        }

        if (j.contains("generated_at")) {
            DES_LOG_INFO(rclcpp::get_logger("des.io.config"), "Building snapshot generated at %s", j.at("generated_at").get<std::string>().c_str());
        } else {
            DES_LOG_WARN(rclcpp::get_logger("des.io.config"), "Building snapshot %s has no generated_at stamp, consider re-baking via ./build_snapshot.sh", filePath.c_str());
        }

        const auto& names = j.at("names");
        const auto& locs  = j.at("locations");
        const bool hasAreas      = j.contains("areas");
        const bool hasFootprints = j.contains("footprints");

        des::LocationMap map;
        for (size_t i = 0; i < names.size() && i < locs.size(); ++i) {
            const std::string name = names.at(i).get<std::string>();
            const auto& l = locs.at(i);
            const des::Point p{ l.at("x").get<double>(), l.at("y").get<double>(), l.value("yaw", 0.0) };

            std::optional<double> area;
            if (hasAreas && i < j.at("areas").size() && !j.at("areas").at(i).is_null()) {
                area = j.at("areas").at(i).get<double>();
            }

            des::Location location(name, p, area);
            if (hasFootprints && i < j.at("footprints").size() && j.at("footprints").at(i).is_array()) {
                for (const auto& v : j.at("footprints").at(i)) {
                    location.m_footprint.push_back(des::Point{v.at(0).get<double>(), v.at(1).get<double>(), 0.0});
                }
            }
            map.emplace(name, std::move(location));
        }
        return map;
    }

    static std::optional<std::pair<std::vector<std::string>, std::vector<std::vector<float>>>>
    loadDistanceMatrix(const std::string& filePath) {
        const auto json = getJson(filePath);
        if (!json.has_value()) {
            return std::nullopt;
        }
        const auto& j = json.value();
        if (!j.contains("names") || !j.contains("mat")) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.config"), "Building snapshot %s missing 'names'/'mat'", filePath.c_str());
            return std::nullopt;
        }
        return std::make_pair(
            j.at("names").get<std::vector<std::string>>(),
            j.at("mat").get<std::vector<std::vector<float>>>());
    }
};
