#pragma once

#include <memory>

#include "../model/context.h"
#include "../model/event.h"
#include "../util/db.h"
#include "../init/config_loader.h"
#include "../observer/ros.h"
#include "../model/event_queue.h"
#include "../util/rnd.h"

class MetricsNode;
constexpr int ONE_HOUR = 3600;
const std::string CONFIG_PATH = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/";

class IAppRunner {
public:
    IAppRunner() = default;
    virtual ~IAppRunner() = default;

    virtual void setupApplication(const std::string& path) = 0;
    virtual void updateConfig() = 0;
    [[nodiscard]] virtual int loadAppState() const = 0;
    virtual void enterPause() const = 0;
    virtual void reset() = 0;
    virtual void shutdown() = 0;

    EventQueue m_eventQueue;
    std::shared_ptr<SimulationContext> m_ctx;

    static SortedEventQueue createMissionQueue(
        const std::shared_ptr<des::SimConfig> &config,
        std::vector<std::shared_ptr<des::Appointment>>& appointments,
        Scheduler& scheduler,
        std::string idleLocation
    ) {
        RCLCPP_DEBUG(rclcpp::get_logger("des_application"), "Start filling event queue");
        SortedEventQueue queue;

        const auto missions = scheduler.simplePlan(appointments, idleLocation);

        for (const auto& mission : missions) {
            const double buffer = config->timeBuffer;
            mission->time = mission->time - buffer;
            queue.push(mission);
        }
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Event queue: (%zu) events inserted (incl. Start and End)", queue.size());
        return queue;
    }

    static std::vector<std::shared_ptr<IEvent>> personArrivalGenerator(
        std::vector<std::shared_ptr<des::Person>> people,
        std::string location
    ) {
        auto events = std::vector<std::shared_ptr<IEvent>> {};
        for (auto& p: people) {
            p->currentRoom = location;
            const auto event = std::make_shared<PersonArrivedEvent>(p->arrivalTime, p);
            events.push_back(event);
        }
        return events;
    }

    static void scheduleOccupancy(
        const des::SimConfig& config,
        std::vector<std::shared_ptr<des::Person>> people
    ) {
        auto sample = [](des::DistributionType type, double mean, double std) -> double {
            switch (type) {
                case des::DistributionType::UNIFORM:     return rnd::uni(mean - std, mean + std);
                case des::DistributionType::EXPONENTIAL:  return rnd::exponential(mean);
                case des::DistributionType::LOG_NORMAL:   return rnd::logNormal(mean, std);
                default:                                  return rnd::normal(mean, std);
            }
        };

        for (auto& p: people) {
            p->arrivalTime = sample(config.arrivalDistribution, config.arrivalMean, config.arrivalStd);
            p->departureTime = sample(config.departureDistribution, config.departureMean, config.departureStd);
        }
    }

protected:
    std::map<std::string, des::Point> m_locationMap;
    std::shared_ptr<des::SimConfig> m_config;
    std::unique_ptr<Scheduler> m_scheduler;
    std::shared_ptr<PathPlannerNode> m_plannerNode;
    std::shared_ptr<MetricsNode> m_metricsNode;
    std::map<std::string, std::vector<std::string>> m_employeeLocations;
    std::vector<std::shared_ptr<des::Appointment>> m_appointments;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::thread m_rosThread;

    static std::vector<std::shared_ptr<des::Appointment>> loadAppointments(const std::string& path) {
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Load Appointments: %s", path.c_str());
        const auto appointments = ConfigLoader::loadAppointmentConfig(path.c_str());
        if (!appointments.has_value()) {
            throw std::runtime_error("Could not load appointments from file");
        }
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Successful loaded %zu appointments", appointments.value().size());
        return appointments.value();
    }

    static std::map<std::string, des::Point> loadPointsOfInterest(const DBConfig& dbCfg) {
        auto db = DBClient(dbCfg);
        if (!db.init()) {
            throw std::runtime_error("Could not connect DB");
        }
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Successful connected to DB");

        const auto pois = db.waypoints();
        if (!pois.has_value()) {
            throw std::runtime_error("Could not load points of interest from file");
        }
        std::map<std::string, des::Point> locationMap;
        for (auto poi : pois.value()) {
            locationMap[poi.m_name] = poi.m_p;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("des_application"), poi);
        }
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Successful loaded %zu points of interest", locationMap.size());
        return locationMap;
    }

    virtual void initROS(const std::vector<std::shared_ptr<rclcpp::Node>> &nodes) {
        // leads to spam messages on lower logger level
        rclcpp::get_logger("event_system_planner_node.rclcpp_action").set_level(rclcpp::Logger::Level::Warn);

        if (!m_plannerNode->isReady()) {
            throw std::runtime_error("Nav2 Planner initialization failed");
        }
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Planner ready");

        m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
        m_rosThread = std::thread([this, nodes] {
            for (const auto& node : nodes) {
                m_executor->add_node(node);
            }
            m_executor->spin();
            for (const auto& node : nodes) {
                m_executor->remove_node(node);
            }
        });
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Launched all ROS Nodes");
    }

    // publishEvents requires queue copy
    static void publishMissions(EventQueue queue, const std::shared_ptr<RosObserver>& publisher) {
        while (!queue.empty()) {
            auto currentEvent  = queue.top();
            const auto mission = std::dynamic_pointer_cast<MissionDispatchEvent>(currentEvent);

            if (mission) {
                publisher->publishMeeting(mission->appointment, mission->time);
            }
            queue.pop();
        }
    }
    

};
