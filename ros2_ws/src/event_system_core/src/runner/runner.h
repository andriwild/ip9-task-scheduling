#pragma once

#include <memory>
#include "../util/log.h"

#include "engine/context.h"
#include "engine/event.h"
#include "../model/occupancy.h"
#include "../sim/ros/path_node.h"
#include "../sim/matrix_planner.h"
#include "../util/db.h"
#include "../init/config_loader.h"
#include "../observer/ros.h"
#include "engine/event_queue.h"
#include "../util/rnd.h"
#include "../util/types.h"
#include "../sim/scheduler.h"
#include "../plugins/order_registry.h"
#include "util/constants.h"

class MetricsNode;
const std::string CONFIG_PATH = CONFIG_DIR;

//TODO: replace with config
const std::string DB_USER     = "wsr_user";
const std::string DB_PASSWORD = "wsr_password";

class IAppRunner {
public:
    static inline std::string s_outDir = "";
    static inline std::string s_runId  = "";

    IAppRunner() : m_db({DB_USER, DB_PASSWORD}) {};

    virtual ~IAppRunner() = default;

    virtual void setupApplication() = 0;
    virtual void updateConfig() = 0;
    [[nodiscard]] virtual int loadAppState() const = 0;
    virtual void enterPause() const = 0;
    virtual void reset() = 0;
    virtual void shutdown() = 0;

    EventQueue m_eventQueue;
    std::shared_ptr<SimulationContext> m_ctx;

    std::shared_ptr<MetricsNode> metricsNode() const {
        return m_metricsNode;
    }

    static SortedEventQueue createMissionQueue(
        des::OrderList& orders,
        Scheduler& scheduler,
        std::string idleLocation
    ) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.runner"), "Start filling event queue");
        SortedEventQueue queue;

        const auto missions = scheduler.createMissionDispatchEvents(orders, idleLocation);

        for (const auto& mission : missions) {
            queue.push(mission);
        }
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Event queue: (%zu) events inserted (incl. Start and End)", queue.size());
        return queue;
    }

    static std::vector<std::shared_ptr<IEvent>> personArrivalGenerator(
        const des::PersonList& people
    ) {
        auto events = std::vector<std::shared_ptr<IEvent>> {};
        for (const auto& p: people) {
            const auto event = std::make_shared<PersonArrivedEvent>(p->arrivalTime, p.get());
            events.push_back(event);
        }
        return events;
    }

    static void scheduleOccupancy(
        const des::SimConfig& config,
        const des::PersonList& people
    ) {
        for (const auto& p: people) {
            des::sampleOccupancy(config, p->rng, 0, *p);
        }
    }

protected:
    des::RoomMap m_rooms;
    std::shared_ptr<des::SimConfig> m_config;
    std::shared_ptr<IPathPlanner> m_planner;
    std::shared_ptr<PathPlannerNode> m_plannerNode;  // null in matrix mode
    std::shared_ptr<MetricsNode> m_metricsNode;
    des::OrderList m_orders;
    std::vector<BackgroundTemplate> m_backgroundTemplates;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::thread m_rosThread;
    DBClient m_db;

    void createPlanner() {
        const auto config = ConfigLoader::loadSimConfig();
        if (!config.has_value()) {
            throw std::runtime_error(
                "Could not load simulation config (base: " + ConfigLoader::baseConfigPath() +
                ", override: " + (ConfigLoader::s_overridePath.empty() ? "none" : ConfigLoader::s_overridePath) + ")");
        }
        if (config->useDistanceMatrix) {
            auto data = ConfigLoader::loadDistanceMatrix(BUILDING_FILE);
            if (!data.has_value()) {
                throw std::runtime_error("use_distance_matrix=true but no matrix in " + BUILDING_FILE);
            }
            m_planner = std::make_shared<MatrixPlanner>(std::move(data->first), std::move(data->second));
            DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Distance source: matrix (%s)", BUILDING_FILE.c_str());
        } else {
            m_plannerNode = std::make_shared<PathPlannerNode>(m_rooms);
            m_planner = m_plannerNode;
            DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Distance source: Nav2 planner");
        }
    }

    void populateEventQueue() {
        if (!m_ctx) {
            throw std::runtime_error("populateEventQueue requires initialized SimulationContext");
        }

        const des::PersonList& people = m_ctx->getAllPersons();

        m_ctx->reseedPersons();
        scheduleOccupancy(*m_config, people);
        m_eventQueue.extend(personArrivalGenerator(people));
        m_eventQueue.extend(createMissionQueue(m_orders, m_ctx->getScheduler(), "IMVS_Dock"));

        const int simStartTime = m_config->simStartTime;
        const int simEndTime   = m_config->simStartTime + m_config->simDuration;
        DES_LOG_DEBUG(rclcpp::get_logger("des.runner"), "Sim window: %d → %d", simStartTime, simEndTime);

        m_eventQueue.push(std::make_shared<SimulationStartEvent>(simStartTime));
        m_eventQueue.push(std::make_shared<SimulationEndEvent>(simEndTime));

        for (const auto& p : people) {
            m_eventQueue.push(std::make_shared<PersonTransitionEvent>(simStartTime, p.get()));
            m_eventQueue.push(std::make_shared<PersonTransitionEvent>(simEndTime, p.get()));
        }

        addEventsFromInterruptGenerators(m_config->appointmentsPath);
        addBackgroundReleaseEvents(simStartTime, simEndTime);

        m_ctx->resetContext(m_eventQueue.getFirstEventTime());

        for (const auto& order : m_orders) {
            m_ctx->publishMissionRegistered(order);
        }

        for (const auto& p : people) {
            m_ctx->setPersonLocation(p->firstName, "OUTDOOR");
        }
    }

    static des::OrderList loadOrders(const std::string& path, const int simStartTime, const int simEndTime) {
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Load orders: %s", path.c_str());
        const auto orders = ConfigLoader::loadOrderConfig(path.c_str(), simStartTime, simEndTime);
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
        const int simStart = m_config->simStartTime;
        const int simEnd   = m_config->simStartTime + m_config->simDuration;

        for (const auto& gen : adHocGenerators.value()) {
            for (int dayBase = 0; dayBase < simEnd; dayBase += SECONDS_PER_DAY) {
                int t = dayBase + gen.from;
                const int winTo = dayBase + gen.to;
                while (t < winTo) {
                    // TODO: not only exponential supported
                    // rnd::exponential takes the mean (sec/event), not the rate (event/sec).
                    const double dt = rnd::exponential(m_ctx->worldRng(), 1.0 / gen.ratePerSecond);
                    t += static_cast<int>(dt);
                    if (t < winTo && t >= simStart && t < simEnd) {
                        nlohmann::json params = gen.params;
                        params["id"] = eventId++;
                        auto orderPtr = OrderRegistry::instance().get(gen.type).fromJson(params);
                        orderPtr->execution = gen.execution;

                        m_eventQueue.push(std::make_shared<OrderArrivalEvent>(t, orderPtr));
                    }
                }
            }
        }
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Successful loaded %zu ad-hoc generators", adHocGenerators->size());
    }

    void addBackgroundReleaseEvents(const int simStartTime, const int simEndTime) {
        const int releaseOffset = simStartTime % SECONDS_PER_DAY;
        int bgId = 300000;
        for (int day = 0, base = 0; base < simEndTime; ++day, base += SECONDS_PER_DAY) {
            const int releaseTime = base + releaseOffset;
            if (releaseTime < simStartTime || releaseTime >= simEndTime) {
                continue;
            }
            for (const auto& tpl : m_backgroundTemplates) {
                const bool due = (tpl.everyNDays <= 0) ? (day == 0) : (day % tpl.everyNDays == 0);
                if (!due) {
                    continue;
                }
                nlohmann::json j = tpl.json;
                j["id"] = bgId++;
                const std::string& type = j.at("type").get_ref<const std::string&>();
                auto order = OrderRegistry::instance().get(type).fromJson(j);
                order->execution = des::ExecutionMode::BACKGROUND;
                m_eventQueue.push(std::make_shared<BackgroundReleaseEvent>(releaseTime, order));
            }
        }
    }

    // Runtime building geometry comes from the generated snapshot (json file), not the DB.
    des::RoomMap loadRooms() {
        auto map = ConfigLoader::loadBuildingSnapshot(BUILDING_FILE);
        if (!map.has_value()) {
            throw std::runtime_error("Could not load building snapshot from " + BUILDING_FILE + ". Generate it first with ./build_snapshot.sh (needs DB + Nav2).");
        }
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Loaded %zu rooms from building snapshot", map.value().size());
        return map.value();
    }

    void mergeRoomTours() {
        const std::string path = ConfigLoader::toursFilePath(m_config->personDetectionRange, m_config->useTspTours);
        const auto merged = ConfigLoader::mergeRoomTours(path, m_rooms);
        if (!merged.has_value()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.runner"), "No room tours at %s; no room can be searched", path.c_str());
            return;
        }
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Merged %zu room tours from %s", merged.value(), path.c_str());

        std::vector<std::string> withoutTour;
        for (const auto& [name, room] : m_rooms) {
            if (room.m_tour.empty()) {
                withoutTour.push_back(name);
            }
        }
        if (!withoutTour.empty()) {
            std::ostringstream oss;
            for (std::size_t i = 0; i < withoutTour.size(); ++i) {
                oss << (i ? ", " : "") << withoutTour[i];
            }
            DES_LOG_WARN(rclcpp::get_logger("des.runner"), "%zu room(s) without tour: %s", withoutTour.size(), oss.str().c_str());
        }
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
