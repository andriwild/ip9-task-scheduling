#pragma once

#include <memory>
#include "../util/log.h"

#include "engine/context.h"
#include "engine/event.h"
#include "../model/occupancy.h"
#include "../sim/matrix_planner.h"
#include "../init/config_loader.h"
#include "engine/event_queue.h"
#include "util/run_state.h"
#include "engine/contracts/i_event.h"
#include "../util/rnd.h"
#include "../util/types.h"
#include "../sim/scheduler.h"
#include "../plugins/order_registry.h"
#include "util/constants.h"
#include "util/perf_profiler.h"

#include "metrics/metrics_reporter.h"
#include "model/sim_config.h"
#include "model/person.h"
#include "model/room.h"
namespace des {

const std::string CONFIG_PATH = CONFIG_DIR;

class IAppRunner {
public:
    static inline std::string s_outDir = "";
    static inline std::string s_runId  = "";

    IAppRunner() = default;

    virtual ~IAppRunner() = default;

    virtual void setupApplication() = 0;
    virtual void updateConfig() = 0;
    [[nodiscard]] virtual RunState loadAppState() const = 0;
    virtual void enterPause() const = 0;
    virtual void onSimulationComplete() = 0;
    virtual void reset() = 0;
    virtual void shutdown() = 0;

    EventQueue m_eventQueue;
    EventList m_protocol;
    std::shared_ptr<SimulationContext> m_ctx;

    metrics::MetricsReporter& reporter() {
        return m_reporter;
    }

    PerfProfiler m_perf;

    static EventList createMissionQueue(
        OrderList& orders,
        Scheduler& scheduler,
        std::string idleLocation
    ) {
        DES_LOG_DEBUG("des.runner", "Start filling event queue");
        const auto missions = scheduler.createMissionDispatchEvents(orders, idleLocation);

        EventList events;
        events.reserve(missions.size());
        for (const auto& mission : missions) {
            events.push_back(mission);
        }
        DES_LOG_INFO("des.runner", "Event queue: (%zu) events inserted (incl. Start and End)", events.size());
        return events;
    }

    static std::vector<std::shared_ptr<IEvent>> personArrivalGenerator(
        const PersonList& people
    ) {
        auto events = std::vector<std::shared_ptr<IEvent>> {};
        for (const auto& p: people) {
            const auto event = std::make_shared<PersonArrivedEvent>(p->arrivalTime, p.get());
            events.push_back(event);
        }
        return events;
    }

    static void scheduleOccupancy(
        const SimConfig& config,
        const PersonList& people
    ) {
        for (const auto& p: people) {
            sampleOccupancy(config, p->rng, 0, *p);
        }
    }

protected:
    RoomMap m_rooms;
    std::shared_ptr<SimConfig> m_config;
    std::shared_ptr<IPathPlanner> m_planner;
    metrics::MetricsReporter m_reporter;
    OrderList m_orders;
    std::vector<BackgroundTemplate> m_backgroundTemplates;

    // Only the ROS-aware runners can serve this, see RosRunner.
    virtual std::shared_ptr<IPathPlanner> createNav2Planner() {
        throw std::runtime_error("use_distance_matrix=false requires the ROS runner");
    }

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
            DES_LOG_INFO("des.runner", "Distance source: matrix (%s)", BUILDING_FILE.c_str());
        } else {
            m_planner = createNav2Planner();
            DES_LOG_INFO("des.runner", "Distance source: Nav2 planner");
        }
    }

    // fill up the event queue with mission and person events
    void populateEventQueue() {
        if (!m_ctx) {
            throw std::runtime_error("populateEventQueue requires initialized SimulationContext");
        }

        const PersonList& people = m_ctx->getAllPersons();

        m_ctx->reseedPersons();
        scheduleOccupancy(*m_config, people);
        m_eventQueue.extend(personArrivalGenerator(people));
        m_eventQueue.extend(createMissionQueue(m_orders, m_ctx->getScheduler(), m_config->dockLocation));

        const int simStartTime = m_config->simStartTime;
        const int simEndTime   = m_config->simStartTime + m_config->simDuration;
        DES_LOG_DEBUG("des.runner", "Sim window: %d → %d", simStartTime, simEndTime);

        m_eventQueue.push(std::make_shared<SimulationStartEvent>(simStartTime));
        m_eventQueue.push(std::make_shared<SimulationEndEvent>(simEndTime));

        for (const auto& p : people) {
            m_eventQueue.push(std::make_shared<PersonTransitionEvent>(simStartTime, p.get()));
            m_eventQueue.push(std::make_shared<PersonTransitionEvent>(simEndTime, p.get()));
        }

        addEventsFromInterruptGenerators(m_config->scenarioPath);
        addBackgroundReleaseEvents(simStartTime, simEndTime);

        m_ctx->resetContext(m_eventQueue.getFirstEventTime());
        m_ctx->setScheduledDispatchPlan(m_orders);

        for (const auto& p : people) {
            m_ctx->setPersonLocation(p->firstName, "OUTDOOR");
        }
    }

    static OrderList loadOrders(const std::string& path, const int simStartTime, const int simEndTime) {
        DES_LOG_INFO("des.runner", "Load orders: %s", path.c_str());
        const auto orders = ConfigLoader::loadOrderConfig(path.c_str(), simStartTime, simEndTime);
        if (!orders.has_value()) {
            throw std::runtime_error("Could not load orders from file");
        }
        DES_LOG_INFO("des.runner", "Successful loaded %zu orders", orders.value().size());
        return orders.value();
    }

    // generate interrupt events on a daily basis within a time window
    void addEventsFromInterruptGenerators(const std::string& path) {
        DES_LOG_INFO("des.runner", "Load ad-hoc generators: %s", path.c_str());

        auto adHocGenerators = ConfigLoader::loadInterruptGenerators(path.c_str());
        int eventId = INTERRUPT_ID_BASE;
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
        DES_LOG_INFO("des.runner", "Successful loaded %zu ad-hoc generators", adHocGenerators->size());
    }

    void addBackgroundReleaseEvents(const int simStartTime, const int simEndTime) {
        const int releaseOffset = simStartTime % SECONDS_PER_DAY;
        int bgId = BACKGROUND_ID_BASE;
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
                order->execution = ExecutionMode::BACKGROUND;
                m_eventQueue.push(std::make_shared<BackgroundReleaseEvent>(releaseTime, order));
            }
        }
    }

    // Runtime building geometry comes from the generated snapshot (json file), not the DB.
    RoomMap loadRooms() {
        auto map = ConfigLoader::loadBuildingSnapshot(BUILDING_FILE);
        if (!map.has_value()) {
            throw std::runtime_error("Could not load building snapshot from " + BUILDING_FILE + ". Generate it first with ./build_snapshot.sh (needs DB + Nav2).");
        }
        DES_LOG_INFO("des.runner", "Loaded %zu rooms from building snapshot", map.value().size());
        return map.value();
    }

    void mergeRoomTours() {

        // The tour raster follows the recognition range: every point of the room must
        // be perceivable from the route, identification then needs a detour.
        std::ostringstream radius;
        radius << m_config->personRecognitionRange;
        const std::string path = CONFIG_DIR + "tours/tours_r" + radius.str() + ".json";

        // add tours through to existing rooms
        const auto merged = ConfigLoader::mergeRoomTours(path, m_rooms);
        if (!merged.has_value()) {
            throw std::runtime_error("Could not load room tours from " + path + ". Generate them first with ./build_tours.sh " + radius.str());
        }
        DES_LOG_INFO("des.runner", "Merged %zu room tours from %s", merged.value(), path.c_str());

        // check if some rooms have no tours
        // docks are stored as room without tours
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
            DES_LOG_WARN("des.runner", "%zu room(s) without tour: %s", withoutTour.size(), oss.str().c_str());
        }
    }


};

}  // namespace des
