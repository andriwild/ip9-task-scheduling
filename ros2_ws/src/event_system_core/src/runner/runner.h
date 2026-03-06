#pragma once

#include <memory>

#include "../model/context.h"
#include "../model/event.h"
#include "../util/db.h"
#include "../init/config_loader.h"

class MetricsNode;
constexpr int ONE_HOUR = 3600;
const std::string CONFIG_PATH = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/";

class IAppRunner {
public:
    IAppRunner() = default;
    virtual ~IAppRunner() = default;

    virtual void setupApplication() = 0;
    virtual void updateConfig() = 0;
    [[nodiscard]] virtual int loadAppState() const = 0;
    virtual void enterPause() const = 0;
    virtual void reset() = 0;
    virtual void shutdown() = 0;
    virtual void setupObservers(bool headless, bool verbose) = 0;

    EventQueue m_eventQueue;
    std::shared_ptr<SimulationContext> m_ctx;

protected:
    std::map<std::string, des::Point> m_locationMap;
    std::shared_ptr<des::SimConfig> m_config;
    std::unique_ptr<Scheduler> m_scheduler;
    std::shared_ptr<PathPlannerNode> m_plannerNode;
    std::shared_ptr<MetricsNode> m_metricsNode;
    std::map<std::string, std::vector<std::string>> m_employeeLocations;
    std::vector<std::shared_ptr<des::Appointment>> m_appointments;

    virtual void loadPointsOfInterest() {
        auto db = DBClient("wsr_user", "wsr_password");
        if (!db.init()) {
            throw std::runtime_error("Could not connect DB");
        }
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Successful connected to DB");

        const auto pois = db.waypoints();
        if (!pois.has_value()) {
            throw std::runtime_error("Could not load points of interest from file");
        }

        for (auto poi : pois.value()) {
            m_locationMap[poi.m_name] = poi.m_p;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("des_application"), poi);
        }
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Successful loaded %zu points of interest", m_locationMap.size());
    }

    virtual void loadEmployeeLocations() {
        RCLCPP_DEBUG(rclcpp::get_logger("des_application"), "Load Employee Locations");
        const auto locations = ConfigLoader::loadEmployeeLocations(CONFIG_PATH + "employee_locations.json");
        if (!locations.has_value()) {
            throw std::runtime_error("Could not load employee from file");
        }
        m_employeeLocations = locations.value();
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Successful loaded employee locations (%zu)", m_employeeLocations.size());
    }

    virtual void loadAppointments(const std::string& path) {
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Load Appointments: %s", m_config->appointmentsPath.c_str());
        const auto appts = ConfigLoader::loadAppointmentConfig(path);
        if (!appts.has_value()) {
            throw std::runtime_error("Could not load appointments from file");
        }
        m_appointments = appts.value();
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Successful loaded %zu appointments", m_appointments.size());
    }

};