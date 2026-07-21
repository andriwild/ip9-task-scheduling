#include "accompany_plugin.h"
#include "../../util/log.h"

#include <memory>

#include <algorithm>

#include "bt_nodes/search.h"
#include "bt_nodes/accompany.h"
#include "bt_nodes/conversation.h"
#include "events/appointment_end_event.h"
#include "sim/scheduler.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "model/sighting.h"
#include "algo/search_instance_builder.h"
#include "algo/op_solver.h"
#include "observer/ros.h"
#include "states.h"

void AccompanyOrderPlugin::registeredNodes(BT::BehaviorTreeFactory& factory) {
    // search
    factory.registerNodeType<IsSearching>("IsSearching");
    factory.registerNodeType<IsScanning>("IsScanning");
    factory.registerNodeType<FoundPerson>("FoundPerson");
    factory.registerNodeType<ScanLocation>("ScanLocation");
    factory.registerNodeType<HasNextLocation>("HasNextLocation");
    factory.registerNodeType<MoveToNextLocation>("MoveToNextLocation");
    factory.registerNodeType<StartAccompanyConversation>("StartAccompanyConversation");
    factory.registerNodeType<AbortSearch>("AbortSearch");

    // accompany
    factory.registerNodeType<IsAccompany>("IsAccompany");
    factory.registerNodeType<ArrivedWithPerson>("ArrivedWithPerson");
    factory.registerNodeType<StartDropOffConversation>("StartDropOffConversation");
    factory.registerNodeType<AbortAccompany>("AbortAccompany");

    // conversation
    factory.registerNodeType<IsConversating>("IsConversating");
    factory.registerNodeType<ConversationFinished>("ConversationFinished");
    factory.registerNodeType<WasConversationSuccessful>("WasConversationSuccessful");
    factory.registerNodeType<IsFoundPersonConversation>("IsFoundPersonConversation");
    factory.registerNodeType<IsDropOffConversation>("IsDropOffConversation");
    factory.registerNodeType<StartAccompanyAction>("StartAccompanyAction");
    factory.registerNodeType<CompleteMissionAction>("CompleteMissionAction");
}

void AccompanyOrderPlugin::onMissionEnd(ISimContext& ctx, des::IOrder& order) {
    auto accompanyOrder = static_cast<AccompanyOrder&>(order);
    const auto& personName = accompanyOrder.personName;
    if (!ctx.hasEmployee(personName)) {
        return;
    }
    auto person = ctx.getPersonByName(personName);

    if (order.state != des::COMPLETED) {
        person->busy = false;
        return;
    }

    int endTime = ctx.getTime() + static_cast<int>(accompanyConfig().appointmentDuration);
    ctx.pushEvent(std::make_shared<AppointmentEndEvent>(endTime, person));
}

namespace {
constexpr int kSearchGraspIterations = 200;
constexpr int kEstimateGraspIterations = 8;
constexpr float kSearchGraspAlpha    = 0.3f;
constexpr int kSearchGraspSeed       = 42;

bool isPersonReachableRoom(const std::string& name, const std::vector<std::string>& excluded) {
    for (const auto& pattern : excluded) {
        if (name.find(pattern) != std::string::npos) {
            return false;
        }
    }
    return true;
}

struct SearchPlan {
    std::vector<std::string> locations;
    double energyWh;
};

std::optional<SearchPlan> planPersonSearch(const ISimContext& ctx, const AccompanyOrder& a, const std::string& startLoc, int graspIterations) {
    const auto person = ctx.getPersonByName(a.personName);
    const auto robot  = ctx.getRobot();

    const auto& excluded = ctx.getConfig()->searchExcludedRooms;
    std::vector<std::string> universe;
    for (const auto& name : ctx.roomNames()) {
        if (isPersonReachableRoom(name, excluded)) {
            universe.push_back(name);
        }
    }
    const auto roomNodes = occupancyProbability(robot->getSightings(), a.personName, person->workplace, universe);

    const auto bat          = robot->m_bat->getStats();
    const double voltage    = robot->m_bat->getVoltage();
    const double currentWh  = bat.soc * bat.capacity * voltage;
    const double capacityWh = bat.capacity * voltage;
    const double reserveWh  = capacityWh * bat.lowThreshold / 100.0;
    const int now           = ctx.getTime();
    const int deadline      = a.deadline.value_or(now);

    const OpBudgets budgets {
        .timeBudget      = static_cast<float>(std::max(0, deadline - now)),
        .energyBudget    = static_cast<float>(currentWh),
        .initialSoc      = static_cast<float>(currentWh),
        .endSocMin       = static_cast<float>(reserveWh),
        .socThreshold    = static_cast<float>(reserveWh),
        .maxEnergy       = static_cast<float>(capacityWh),
        .chargeTimePerWh = 1e9f,
        .chargeTimePerWhTapered = 1e9f,
        .cvEnergy        = static_cast<float>(capacityWh),
    };

    auto instance = buildSearchInstance(ctx, roomNodes, startLoc, a.roomName, budgets);
    if (!instance) {
        return std::nullopt;
    }
    const auto route = op_solver::grasp(*instance, graspIterations, kSearchGraspAlpha, kSearchGraspSeed);
    if (route.empty()) {
        return std::nullopt;
    }

    const auto cfg           = ctx.getConfig();
    const double driveWhPerM = cfg->energyConsumptionDrive / (3600.0 * cfg->robotSpeed);
    double energyWh          = instance->routeDriveDistance(route) * driveWhPerM;
    std::vector<std::string> locations;
    for (const int idx : route) {
        energyWh += instance->node(idx).serviceEnergy;
        locations.push_back(instance->node(idx).name);
    }
    return SearchPlan{ std::move(locations), energyWh };
}
}

void AccompanyOrderPlugin::onMissionStart(ISimContext& ctx, des::IOrder& order) {
    auto& accompanyOrder = static_cast<AccompanyOrder&>(order);
    order.state          = des::MissionState::IN_PROGRESS;
    const auto person    = ctx.getPersonByName(accompanyOrder.personName);

    std::vector<std::string> locations;
    if (auto plan = planPersonSearch(ctx, accompanyOrder, ctx.getRobot()->getLocation(), kSearchGraspIterations)) {
        locations = std::move(plan->locations);
    }
    if (locations.empty()) {
        locations.push_back(person->workplace);
    }

    accompanyOrder.plannedSearch = locations;
    ctx.changeRobotState(std::make_unique<SearchState>(locations));
}

void AccompanyOrderPlugin::onStartDriveEvent(ISimContext& ctx, des::IOrder& order) {
    // not implemented
};

void AccompanyOrderPlugin::onStopDriveEvent(ISimContext& ctx, des::IOrder& order) {
    auto accompany = std::dynamic_pointer_cast<AccompanyOrder>(ctx.getOrderPtr());
    if (dynamic_cast<AccompanyState*>(ctx.getRobot()->getState()) != nullptr && accompany) {
        const auto& personName = accompany->personName;
        if (ctx.hasEmployee(personName)) {
            auto person = ctx.getPersonByName(personName);
            const auto& arrivalLocation = ctx.getRobot()->getLocation();
            ctx.setPersonLocation(personName, arrivalLocation);
            ctx.pushEvent(std::make_shared<PersonAccompanyArrivedEvent>(ctx.getTime(), person, arrivalLocation));
        }
    }
};

des::OrderPtr AccompanyOrderPlugin::fromJson(const nlohmann::json& j) const {
    auto o = std::make_shared<AccompanyOrder>();
    o->id          = j.at("id");
    o->type        = "accompany";
    o->deadline    = j.at("appointmentTime");
    o->description = j.value("description", "");
    o->personName  = j.at("personName");
    o->roomName    = j.at("roomName");
    o->execution   = des::ExecutionMode::SCHEDULED;
    return o;
}

namespace {
double meetingViaWorkplace(const Scheduler& sched, const std::string& workplace, const std::string& startPos, const std::string& goalPos) {
    const double accompanySpd  = accompanyConfig().accompanySpeed;
    const double searchTime    = sched.robotDriveTime(startPos, workplace);
    const double scanTime      = sched.getScanTime(workplace);
    const double accompanyTime = sched.getDriveTime(workplace, goalPos, accompanySpd);
    return searchTime + accompanyTime + scanTime;
}
}

int AccompanyOrderPlugin::planDispatchTime(const des::IOrder& order, const Scheduler& s, const std::string& startPos) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);
    (void)startPos;
    return *a.deadline - s.timeBuffer();
}

bool AccompanyOrderPlugin::isFeasible(const des::IOrder& order, const ISimContext& context) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);

    const auto robotLocation = context.getRobot()->getLocation();
    const auto person        = context.getPersonByName(a.personName);
    const double missionDuration = meetingViaWorkplace(context.getScheduler(), person->workplace, robotLocation, a.roomName);
    const int slack = static_cast<int>(order.deadline.value() - missionDuration - context.getTime());
    if (slack >= 0) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany"), "Mission %u is feasible", order.id);
        return true;
    }
    DES_LOG_INFO(rclcpp::get_logger("des.plugin.accompany"),
                 "Mission %u (%s -> %s) infeasible: deadline %d, optimistic mission %.0fs from %s, now %d → slack %ds",
                 order.id, a.personName.c_str(), a.roomName.c_str(),
                 *order.deadline, missionDuration, robotLocation.c_str(),
                 context.getTime(), slack);
    return false;
}

std::optional<std::string> AccompanyOrderPlugin::targetLocation(const des::IOrder& order) const {
    return static_cast<const AccompanyOrder&>(order).roomName;
}

double AccompanyOrderPlugin::estimateMissionEnergy(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) const {
    const auto& a     = static_cast<const AccompanyOrder&>(order);
    const auto& cfg   = *context.getConfig();
    const auto& sched = context.getScheduler();

    double searchWh;
    if (auto plan = planPersonSearch(context, a, startLocation, kEstimateGraspIterations)) {
        searchWh = plan->energyWh;
    } else {
        const auto person    = context.getPersonByName(a.personName);
        const double meeting = meetingViaWorkplace(sched, person->workplace, startLocation, a.roomName);
        searchWh             = meeting * cfg.energyConsumptionDrive / 3600.0;
    }

    const double appointmentWh = accompanyConfig().appointmentDuration * cfg.energyConsumptionBase / 3600.0;
    const double driveBackWh   = sched.robotDriveTime(a.roomName, cfg.dockLocation) * cfg.energyConsumptionDrive / 3600.0;
    return searchWh + appointmentWh + driveBackWh;
}

void AccompanyOrderPlugin::publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);
    observer.publishMeeting(
        a.id,
        startTime,
        a.deadline.value_or(0),
        static_cast<int>(a.state),
        kTypeName,
        a.personName,
        a.roomName,
        a.description,
        static_cast<int>(a.execution));
}
