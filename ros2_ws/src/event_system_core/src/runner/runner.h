#pragma once

#include <memory>

#include "../model/context.h"
#include "../model/event.h"
#include "../sim/ros/path_node.h"
#include "../util/db.h"
#include "../init/config_loader.h"
#include "../observer/ros.h"
#include "../model/event_queue.h"
#include "../util/rnd.h"
#include "../util/types.h"

class MetricsNode;
const std::string CONFIG_PATH = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/";
const std::string DB_USER = "wsr_user";
const std::string DB_PASSWORD = "wsr_password";

class IAppRunner {
public:
    IAppRunner() : m_db({DB_USER, DB_PASSWORD}) {
    };

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
        des::PersonList people
    ) {
        auto events = std::vector<std::shared_ptr<IEvent>> {};
        for (auto& p: people) {
            const auto event = std::make_shared<PersonArrivedEvent>(p->arrivalTime, p);
            events.push_back(event);
        }
        return events;
    }

    static void scheduleOccupancy(
        const des::SimConfig& config,
        des::PersonList people,
        std::mt19937& rng
    ) {
        auto sample = [&rng](des::DistributionType type, double mean, double std) -> double {
            switch (type) {
                case des::DistributionType::UNIFORM:        return rnd::uni(rng, mean - std, mean + std);
                case des::DistributionType::EXPONENTIAL:    return rnd::exponential(rng, mean);
                case des::DistributionType::LOG_NORMAL:     return rnd::logNormal(rng, mean, std);
                default:                                    return rnd::normal(rng, mean, std);
            }
        };

        for (auto& p: people) {
            p->arrivalTime = sample(config.arrivalDistribution, config.arrivalMean, config.arrivalStd);
            p->departureTime = sample(config.departureDistribution, config.departureMean, config.departureStd);
        }
    }

protected:
    std::map<std::string, des::Point> m_locationMap;
    des::SearchAreaMap m_searchAreaMap;
    std::shared_ptr<des::SimConfig> m_config;
    std::shared_ptr<PathPlannerNode> m_plannerNode;
    std::shared_ptr<MetricsNode> m_metricsNode;
    std::map<std::string, std::shared_ptr<des::Person>> m_employeeLocations;
    std::vector<std::shared_ptr<des::Appointment>> m_appointments;
    std::optional<des::PersonList> m_people;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::thread m_rosThread;
    DBClient m_db;

    void populateEventQueue() {
        if (!m_ctx) {
            throw std::runtime_error("populateEventQueue requires initialized SimulationContext");
        }

        scheduleOccupancy(*m_config, m_people.value(), m_ctx->m_rng);
        m_eventQueue.extend(personArrivalGenerator(m_people.value()));
        m_eventQueue.extend(createMissionQueue(m_config, m_appointments, m_ctx->getScheduler(), "IMVS_Dock"));

        int firstEventTime = m_eventQueue.getFirstEventTime() - ONE_HOUR;
        int lastEventTime  = m_eventQueue.getLastEventTime();

        auto latest = std::max_element(m_people.value().begin(), m_people.value().end(),
            [](const auto& a, const auto& b) { return a->departureTime < b->departureTime; });
        int lastDepartureTime = (*latest)->departureTime;

        RCLCPP_DEBUG(rclcpp::get_logger("des_application"), "Event time range from %d to %d", firstEventTime, lastEventTime);

        int simEndTime = std::max(lastEventTime, lastDepartureTime) + ONE_HOUR;
        m_eventQueue.push(std::make_shared<SimulationStartEvent>(firstEventTime));
        m_eventQueue.push(std::make_shared<SimulationEndEvent>(simEndTime));

        for (auto& p : m_people.value()) {
            m_eventQueue.push(std::make_shared<PersonTransitionEvent>(firstEventTime, p));
            m_eventQueue.push(std::make_shared<PersonTransitionEvent>(simEndTime, p));
        }

        m_ctx->resetContext(m_eventQueue.getFirstEventTime());

        for (auto& p : m_people.value()) {
            m_ctx->setPersonLocation(p->firstName, "OUTDOOR");
        }
    }

    static std::vector<std::shared_ptr<des::Appointment>> loadAppointments(const std::string& path) {
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Load Appointments: %s", path.c_str());
        const auto appointments = ConfigLoader::loadAppointmentConfig(path.c_str());
        if (!appointments.has_value()) {
            throw std::runtime_error("Could not load appointments from file");
        }
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Successful loaded %zu appointments", appointments.value().size());
        return appointments.value();
    }

    des::SearchAreaMap loadSearchAreas() {
        const auto areas = m_db.allAreas();
        if (!areas.has_value()) {
            throw std::runtime_error("Could not load search areas from DB");
        }
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "Successful loaded %zu search areas", areas.value().size());
        return areas.value();
    }

    std::map<std::string, des::Point> loadPointsOfInterest() {
        const auto pois = m_db.waypoints();
        if (!pois.has_value()) {
            throw std::runtime_error("Could not load points of interest from DB");
        }
        std::map<std::string, des::Point> locationMap;
        for (const auto& poi : pois.value()) {
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
