#include "accompany_plugin.h"
#include "../../util/log.h"

#include <memory>

#include <algorithm>
#include <cmath>

#include "bt_nodes/search.h"
#include "bt_nodes/accompany.h"
#include "bt_nodes/conversation.h"
#include "events/appointment_end_event.h"
#include "sim/scheduler.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/sighting.h"
#include "algo/search/search_instance_builder.h"
#include "algo/search/search_reward.h"
#include "algo/search/search_solver.h"
#include "observer/ros.h"
#include "search_exclusion.h"
#include "states.h"

void AccompanyOrderPlugin::registeredNodes(BT::BehaviorTreeFactory& factory) {
    // search
    factory.registerNodeType<IsSearching>("IsSearching");
    factory.registerNodeType<FoundPerson>("FoundPerson");
    factory.registerNodeType<HasScanPoint>("HasScanPoint");
    factory.registerNodeType<ScanNextPoint>("ScanNextPoint");
    factory.registerNodeType<HasNextLocation>("HasNextLocation");
    factory.registerNodeType<MoveToNextLocation>("MoveToNextLocation");
    factory.registerNodeType<StartAccompanyConversation>("StartAccompanyConversation");
    factory.registerNodeType<ReportSearchAbort>("ReportSearchAbort");

    // accompany
    factory.registerNodeType<IsAccompany>("IsAccompany");
    factory.registerNodeType<HasArrived>("HasArrived");
    factory.registerNodeType<ArrivedWithPerson>("ArrivedWithPerson");
    factory.registerNodeType<StartDropOffConversation>("StartDropOffConversation");

    // conversation
    factory.registerNodeType<IsConversating>("IsConversating");
    factory.registerNodeType<ConversationFinished>("ConversationFinished");
    factory.registerNodeType<WasConversationSuccessful>("WasConversationSuccessful");
    factory.registerNodeType<IsFoundPersonConversation>("IsFoundPersonConversation");
    factory.registerNodeType<IsDropOffConversation>("IsDropOffConversation");
    factory.registerNodeType<StartAccompanyAction>("StartAccompanyAction");
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

struct SearchPlan {
    std::vector<std::string> locations;
    double energyWh;
    double durationSec;
};

std::optional<SearchPlan> planPersonSearch(const ISimContext& ctx, const AccompanyOrder& a, const std::string& startLoc) {
    const auto person = ctx.getPersonByName(a.personName);
    const auto robot  = ctx.getRobot();

    const auto& excluded = ctx.getConfig()->searchExcludedRooms;
    std::vector<std::string> universe;
    std::vector<des::RoomType> universeTypes;
    for (const auto& name : ctx.roomNames()) {
        if (!isSearchExcluded(excluded, name)) {
            universe.push_back(name);
            universeTypes.push_back(ctx.room(name).m_roomType);
        }
    }
    const auto strategy = ctx.getConfig()->searchRewardStrategy;
    const auto roomNodes = strategy == des::SearchRewardStrategy::FREQUENCY
        ? frequencyReward(robot->getSightings(), a.personName, universe)
        : occupancyProbability(robot->getSightings(), a.personName, person->workplace, universe, universeTypes, person->roles, ctx.getConfig()->searchRolePrior);

    const auto bat          = robot->batteryStats();
    const double voltage    = robot->batteryVoltage();
    const double currentWh  = bat.soc * bat.capacity * voltage;
    const double capacityWh = bat.capacity * voltage;
    const double reserveWh  = capacityWh * bat.lowThreshold / 100.0;
    const double spendableWh = currentWh - reserveWh;
    const int now           = ctx.getTime();
    const int deadline      = a.deadline.value_or(now);

    const OpBudgets budgets {
        .timeBudget      = static_cast<float>(std::max(0, deadline - now)),
        .energyBudget    = static_cast<float>(std::max(spendableWh, kMinEnergyBudgetWh)),
        .initialSoc      = static_cast<float>(currentWh),
        .endSocMin       = static_cast<float>(reserveWh),
        .socThreshold    = static_cast<float>(reserveWh),
        .maxEnergy       = static_cast<float>(capacityWh),
        .chargeTimePerWh = 1e9f,
        .chargeTimePerWhTapered = 1e9f,
        .cvEnergy        = static_cast<float>(capacityWh),
    };

    auto instance = buildSearchInstance(ctx, ctx, *ctx.getConfig(), roomNodes, startLoc, a.roomName, budgets);
    if (!instance) {
        return std::nullopt;
    }
    const auto route = op_solver::greedySearchOrder(*instance);
    if (route.empty()) {
        return std::nullopt;
    }

    const auto cfg           = ctx.getConfig();
    const double driveWhPerM = cfg->energyConsumptionDrive / (3600.0 * cfg->robotSpeed);
    const double driveDist   = instance->routeDriveDistance(route);
    double energyWh          = driveDist * driveWhPerM;
    double durationSec       = cfg->robotSpeed > 0.0 ? driveDist / cfg->robotSpeed : 0.0;
    std::vector<std::string> locations;
    for (const int idx : route) {
        energyWh += instance->node(idx).serviceEnergy;
        durationSec += instance->node(idx).serviceTime;
        locations.push_back(instance->node(idx).name);
    }
    return SearchPlan{ std::move(locations), energyWh, durationSec };
}
}

void AccompanyOrderPlugin::onMissionStart(ISimContext& ctx, des::IOrder& order) {
    auto& accompanyOrder = static_cast<AccompanyOrder&>(order);
    order.state          = des::MissionState::IN_PROGRESS;
    const auto person    = ctx.getPersonByName(accompanyOrder.personName);

    std::vector<std::string> locations;
    if (auto plan = planPersonSearch(ctx, accompanyOrder, ctx.getRobot()->getLocation())) {
        locations = std::move(plan->locations);
    }
    if (locations.empty()) {
        locations.push_back(person->workplace);
    }

    accompanyOrder.plannedSearch = locations;
    accompanyOrder.remainingSearch = locations;
    accompanyOrder.scanIndex = 0;
    accompanyOrder.phase = AccompanyPhase::SEARCH;
    ctx.changeRobotState(std::make_unique<SearchState>());
}

void AccompanyOrderPlugin::onMissionResume(ISimContext& ctx, des::IOrder& order) {
    auto& accompanyOrder = static_cast<AccompanyOrder&>(order);
    switch (accompanyOrder.phase) {
        case AccompanyPhase::SEARCH: {
            ctx.changeRobotState(std::make_unique<SearchState>());
            break;
        }
        case AccompanyPhase::ACCOMPANY: {
            ctx.changeRobotState(std::make_unique<AccompanyState>());
            break;
        }
        case AccompanyPhase::CONVERSATE_FOUND: {
            ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));
            break;
        }
        case AccompanyPhase::CONVERSATE_DROPOFF: {
            ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::DROP_OFF));
            break;
        }
        case AccompanyPhase::NONE: {
            break;
        }
    }
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
struct MeetingEstimate {
    double driveTime;
    double talkTime;

    double total() const {
        return driveTime + talkTime;
    }
};

MeetingEstimate meetingViaWorkplace(const Scheduler& sched, const std::string& workplace, const std::string& startPos, const std::string& goalPos) {
    const auto& cfg            = accompanyConfig();
    const double searchTime    = sched.robotDriveTime(startPos, workplace);
    const double scanTime      = sched.getScanTime(workplace);
    const double accompanyTime = sched.getDriveTime(workplace, goalPos, cfg.accompanySpeed);
    return { searchTime + scanTime + accompanyTime, 2.0 * cfg.conversationDurationMean };
}
}

int AccompanyOrderPlugin::planDispatchTime(const des::IOrder& order, const Scheduler& s, const std::string& startPos) const {
    const auto& mission = static_cast<const AccompanyOrder&>(order);
    return *mission.deadline - s.timeBuffer();
}

bool AccompanyOrderPlugin::isFeasible(const des::IOrder& order, const ISimContext& context) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);

    const auto robotLocation     = context.getRobot()->getLocation();
    const auto person            = context.getPersonByName(a.personName);
    const double missionDuration = meetingViaWorkplace(context.getScheduler(), person->workplace, robotLocation, a.roomName).total();

    const int slack = static_cast<int>(std::floor(order.deadline.value() - missionDuration - context.getTime()));

    if (slack >= 0) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany"), "Mission %u is feasible", order.id);
        return true;
    }
    DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany"),
                 "Mission %u (%s -> %s) infeasible: deadline %d, optimistic mission %.0fs from %s, now %d → slack %ds",
                 order.id, a.personName.c_str(), a.roomName.c_str(),
                 *order.deadline, missionDuration, robotLocation.c_str(),
                 context.getTime(), slack);
    return false;
}

std::optional<std::string> AccompanyOrderPlugin::targetLocation(const des::IOrder& order) const {
    return static_cast<const AccompanyOrder&>(order).roomName;
}

std::string AccompanyOrderPlugin::outcomeDetail(const des::IOrder& order) const {
    switch (static_cast<const AccompanyOrder&>(order).abortReason) {
        case SearchAbortReason::OUTSIDE:                 return "person outside";
        case SearchAbortReason::IN_BUILDING_FINDABLE:    return "missed in building";
        case SearchAbortReason::IN_BUILDING_UNREACHABLE: return "unreachable room";
        default:                                         return {};
    }
}

double AccompanyOrderPlugin::estimateMissionEnergy(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) const {
    const auto& a     = static_cast<const AccompanyOrder&>(order);
    const auto& cfg   = *context.getConfig();
    const auto& sched = context.getScheduler();

    double searchWh;
    if (auto plan = planPersonSearch(context, a, startLocation)) {
        searchWh = plan->energyWh;
    } else {
        const auto person  = context.getPersonByName(a.personName);
        const auto meeting = meetingViaWorkplace(sched, person->workplace, startLocation, a.roomName);
        searchWh           = (meeting.driveTime * cfg.energyConsumptionDrive + meeting.talkTime * cfg.energyConsumptionBase) / 3600.0;
    }

    const double appointmentWh = accompanyConfig().appointmentDuration * cfg.energyConsumptionBase / 3600.0;
    const double driveBackWh   = sched.robotDriveTime(a.roomName, cfg.dockLocation) * cfg.energyConsumptionDrive / 3600.0;
    return searchWh + appointmentWh + driveBackWh;
}

double AccompanyOrderPlugin::estimateMissionDuration(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) const {
    const auto& a     = static_cast<const AccompanyOrder&>(order);
    const auto& cfg   = *context.getConfig();
    const auto& sched = context.getScheduler();

    double searchSec;
    if (auto plan = planPersonSearch(context, a, startLocation)) {
        searchSec = plan->durationSec;
    } else {
        const auto person = context.getPersonByName(a.personName);
        searchSec         = meetingViaWorkplace(sched, person->workplace, startLocation, a.roomName).total();
    }

    const double driveBackSec = sched.robotDriveTime(a.roomName, cfg.dockLocation);
    return searchSec + accompanyConfig().appointmentDuration + driveBackSec;
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
