#pragma once

#include <memory>
#include "../util/log.h"

#include "../model/context.h"
#include "../model/event.h"
#include "../sim/ros/path_node.h"
#include "../sim/matrix_planner.h"
#include "../util/db.h"
#include "../init/config_loader.h"
#include "../observer/ros.h"
#include "../model/event_queue.h"
#include "../util/rnd.h"
#include "../util/types.h"
#include "../sim/scheduler.h"
#include "../plugins/order_registry.h"

class MetricsNode;
const std::string CONFIG_PATH = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/";
const std::string DB_USER     = "wsr_user";
const std::string DB_PASSWORD = "wsr_password";

inline des::LocationMap loadLocationsFromDB(DBClient& db) {
    const auto pois = db.waypoints();
    if (!pois.has_value()) {
        throw std::runtime_error("Could not load points of interest from DB");
    }
    des::LocationMap locationMap;
    for (const auto& poi : pois.value()) {
        locationMap.emplace(poi.m_name, poi);
    }
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Successful loaded %zu points of interest", locationMap.size());

    const auto areas = db.allAreas();
    if (!areas.has_value()) {
        throw std::runtime_error("Could not load location areas from DB");
    }
    size_t matched = 0;
    for (const auto& [name, area] : areas.value()) {
        const auto it = locationMap.find(name);
        if (it == locationMap.end()) {
            DES_LOG_WARN(rclcpp::get_logger("des.runner"), "Search zone '%s' has no point of interest; area dropped", name.c_str());
            continue;
        }
        it->second.m_area = area;
        ++matched;
    }
    DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Merged %zu/%zu areas into locations", matched, areas.value().size());

    for (const auto& [_, loc] : locationMap) {
        DES_LOG_DEBUG_STREAM(rclcpp::get_logger("des.runner"), loc);
    }
    return locationMap;
}

class IAppRunner {
public:
    IAppRunner() : m_db({DB_USER, DB_PASSWORD}) {};

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
        des::OrderList& orders,
        Scheduler& scheduler,
        std::string idleLocation
    ) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.runner"), "Start filling event queue");
        SortedEventQueue queue;

        const auto missions = scheduler.simplePlan(orders, idleLocation);

        for (const auto& mission : missions) {
            const double buffer = config->timeBuffer;
            mission->time = mission->time - buffer;
            queue.push(mission);
        }
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Event queue: (%zu) events inserted (incl. Start and End)", queue.size());
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
    des::LocationMap m_locationMap;
    std::shared_ptr<des::SimConfig> m_config;
    std::shared_ptr<IPathPlanner> m_planner;
    std::shared_ptr<PathPlannerNode> m_plannerNode;  // null in matrix mode
    std::shared_ptr<MetricsNode> m_metricsNode;

    void createPlanner() {
        const bool useMatrix = ConfigLoader::loadSimConfig().value_or(des::SimConfig{}).useDistanceMatrix;
        if (useMatrix) {
            auto data = ConfigLoader::loadDistanceMatrix(BUILDING_FILE);
            if (!data.has_value()) {
                throw std::runtime_error("use_distance_matrix=true but no matrix in " + BUILDING_FILE);
            }
            m_planner = std::make_shared<MatrixPlanner>(std::move(data->first), std::move(data->second));
            DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Distance source: matrix (%s)", BUILDING_FILE.c_str());
        } else {
            m_plannerNode = std::make_shared<PathPlannerNode>(m_locationMap);
            m_planner = m_plannerNode;
            DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Distance source: Nav2 planner");
        }
    }
    std::map<std::string, std::shared_ptr<des::Person>> m_employeeLocations;
    des::OrderList m_orders;
    des::OrderList m_backgroundOrders;
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
        m_eventQueue.extend(createMissionQueue(m_config, m_orders, m_ctx->getScheduler(), "IMVS_Dock"));

        const int simStartTime = m_config->simStartTime;
        const int simEndTime   = m_config->simEndTime;
        DES_LOG_DEBUG(rclcpp::get_logger("des.runner"), "Sim window: %d → %d", simStartTime, simEndTime);

        m_eventQueue.push(std::make_shared<SimulationStartEvent>(simStartTime));
        m_eventQueue.push(std::make_shared<SimulationEndEvent>(simEndTime));

        for (auto& p : m_people.value()) {
            m_eventQueue.push(std::make_shared<PersonTransitionEvent>(simStartTime, p));
            m_eventQueue.push(std::make_shared<PersonTransitionEvent>(simEndTime, p));
        }

        addEventsFromInterruptGenerators(m_config->appointmentsPath);

        m_ctx->resetContext(m_eventQueue.getFirstEventTime());

        // Background tasks live in the background mission pool, not in the event queue
        for (const auto& order : m_backgroundOrders) {
            m_ctx->addBackgroundMission(order);
        }

        for (const auto& order : m_orders) {
            m_ctx->publishMissionRegistered(order);
        }
        for (const auto& order : m_backgroundOrders) {
            m_ctx->publishMissionRegistered(order);
        }

        for (auto& p : m_people.value()) {
            m_ctx->setPersonLocation(p->firstName, "OUTDOOR");
        }
    }

    static des::OrderList loadOrders(const std::string& path) {
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Load orders: %s", path.c_str());
        const auto orders = ConfigLoader::loadOrderConfig(path.c_str());
        if (!orders.has_value()) {
            throw std::runtime_error("Could not load orders from file");
        }
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Successful loaded %zu orders", orders.value().size());
        return orders.value();
    }

    void addEventsFromInterruptGenerators(const std::string& path) {
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Load ad-hoc generators: %s", path.c_str());
        auto adHocGenerators = ConfigLoader::loadInterruptGenerators(path.c_str());
        int eventId = 100000;

        for (const auto& gen : adHocGenerators.value()) {
            int t = gen.from;
            while (t < gen.to) {
                // TODO: not only exponential supported
                // rnd::exponential takes the mean (sec/event), not the rate (event/sec).
                const double dt = rnd::exponential(m_ctx->rng(), 1.0 / gen.ratePerSecond);
                t += static_cast<int>(dt);
                if (t < gen.to) {
                    nlohmann::json params = gen.params;
                    params["id"] = eventId++;
                    auto orderPtr = OrderRegistry::instance().get(gen.type).fromJson(params);
                    orderPtr->execution = gen.execution;

                    m_eventQueue.push(std::make_shared<OrderArrivalEvent>(t, orderPtr));
                }
            }
        }
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Successful loaded %zu ad-hoc generators", adHocGenerators->size());
    }

    // Runtime building geometry comes from the generated snapshot, not the DB.
    des::LocationMap loadLocations() {
        auto map = ConfigLoader::loadBuildingSnapshot(BUILDING_FILE);
        if (!map.has_value()) {
            throw std::runtime_error("Could not load building snapshot from " + BUILDING_FILE +
                                     ". Generate it first with ./build_snapshot.sh (needs DB + Nav2).");
        }
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Loaded %zu locations from building snapshot", map.value().size());
        return map.value();
    }

    virtual void initROS(const std::vector<std::shared_ptr<rclcpp::Node>> &nodes) {
        // leads to spam messages on lower logger level
        rclcpp::get_logger("event_system_planner_node.rclcpp_action").set_level(rclcpp::Logger::Level::Warn);

        if (m_plannerNode) {
            if (!m_plannerNode->isReady()) {
                throw std::runtime_error("Nav2 Planner initialization failed");
            }
            DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Planner ready");
        }

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
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Launched all ROS Nodes");
    }

};
