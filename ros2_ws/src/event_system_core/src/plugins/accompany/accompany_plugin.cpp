#include "accompany_plugin.h"
#include "../../util/log.h"

#include <memory>

#include <algorithm>
#include <cmath>
#include <ranges>

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
#include "search_exclusion.h"
#include "states.h"

namespace des {

void AccompanyOrderPlugin::registeredNodes(BT::BehaviorTreeFactory& factory) {
    // search
    factory.registerNodeType<IsSearching>("IsSearching");
    factory.registerNodeType<FoundPerson>("FoundPerson");
    factory.registerNodeType<HasScanPoint>("HasScanPoint");
    factory.registerNodeType<ScanNextPoint>("ScanNextPoint");
    factory.registerNodeType<HasNextLocation>("HasNextLocation");
    factory.registerNodeType<MoveToNextLocation>("MoveToNextLocation");
    factory.registerNodeType<StartAccompanyConversation>("StartAccompanyConversation");
    factory.registerNodeType<HasPendingAsk>("HasPendingAsk");
    factory.registerNodeType<StartAskConversation>("StartAskConversation");
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
    factory.registerNodeType<IsConversationKind>("IsConversationKind");
    factory.registerNodeType<ApplyDirections>("ApplyDirections");
    factory.registerNodeType<ResumeSearchAfterAsk>("ResumeSearchAfterAsk");
    factory.registerNodeType<StartAccompanyAction>("StartAccompanyAction");
}

void AccompanyOrderPlugin::onMissionEnd(ISimContext& ctx, IOrder& order) {
    auto accompanyOrder = static_cast<AccompanyOrder&>(order);
    const auto& personName = accompanyOrder.personName;
    if (!ctx.hasEmployee(personName)) {
        return;
    }
    auto person = ctx.getPersonByName(personName);

    if (order.state != COMPLETED) {
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
    const auto cfg    = ctx.getConfig();
    const auto person = ctx.getPersonByName(a.personName);
    const auto robot  = ctx.getRobot();

    const auto allNames = ctx.roomNames();
    auto searchable = allNames
        | std::views::filter([&](const std::string& name) { return !isSearchExcluded(ctx, name); })
        | std::views::transform([&](const std::string& name) { return SearchRoom{ name, ctx.room(name).m_roomType }; })
        | std::views::common;
    const std::vector<SearchRoom> searchableRooms(searchable.begin(), searchable.end());

    const auto roomNodes = [&]() {
        switch (cfg->searchRewardStrategy) {
            case SearchRewardStrategy::RANDOM: {
                return randomReward(ctx.robotRng(), searchableRooms);
            }
            case SearchRewardStrategy::RANDOM_SECTOR: {
                return sectorReward(ctx.robotRng(), searchableRooms, person->workplace);
            }
            case SearchRewardStrategy::UNIFORM: {
                return uniformReward(searchableRooms);
            }
            case SearchRewardStrategy::FREQUENCY: {
                return frequencyReward(robot->getSightings(), a.personName, searchableRooms);
            }
            case SearchRewardStrategy::TRUE_DISTRIBUTION: {
                return trueDistributionReward(person->workplace, person->roomLabels,
                                              cfg->searchTrueWorkplaceShare, cfg->searchTrueDistribution,
                                              searchableRooms);
            }
            default: {
                return occupancyProbability(robot->getSightings(), a.personName, person->workplace, searchableRooms, cfg->searchPriorWeight, static_cast<float>(cfg->searchWorkplacePrior));
            }
        }
    }();

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
        .chargeTimePerWh = kChargingPricedOut,
        .chargeTimePerWhTapered = kChargingPricedOut,
        .cvEnergy        = static_cast<float>(capacityWh),
    };

    auto instance = buildSearchInstance(ctx, ctx, *cfg, roomNodes, startLoc, a.roomName, budgets);
    if (!instance) {
        return std::nullopt;
    }
    const auto route = op_solver::greedySearchOrder(*instance);
    if (route.empty()) {
        return std::nullopt;
    }

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

void AccompanyOrderPlugin::onMissionStart(ISimContext& ctx, IOrder& order) {
    auto& accompanyOrder = static_cast<AccompanyOrder&>(order);
    order.state          = OrderState::IN_PROGRESS;
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
    accompanyOrder.scanRoom.clear();
    accompanyOrder.scanQueue.clear();
    accompanyOrder.pendingAsk.clear();
    accompanyOrder.phase = AccompanyPhase::SEARCH;
    ctx.changeRobotState(std::make_unique<SearchState>());
}

void AccompanyOrderPlugin::onMissionResume(ISimContext& ctx, IOrder& order) {
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
            ctx.changeRobotState(std::make_unique<ConversationState>(ConversationKind::FOUND_PERSON));
            break;
        }
        case AccompanyPhase::CONVERSATE_DROPOFF: {
            ctx.changeRobotState(std::make_unique<ConversationState>(ConversationKind::DROP_OFF));
            break;
        }
        case AccompanyPhase::CONVERSATE_ASK: {
            ctx.changeRobotState(std::make_unique<ConversationState>(ConversationKind::ASK_DIRECTIONS));
            break;
        }
        case AccompanyPhase::NONE: {
            break;
        }
    }
}

void AccompanyOrderPlugin::onConversationStart(ISimContext& /*ctx*/, IOrder& order, const ConversationKind kind) {
    auto& accompanyOrder = static_cast<AccompanyOrder&>(order);
    switch (kind) {
        case ConversationKind::FOUND_PERSON: {
            accompanyOrder.phase = AccompanyPhase::CONVERSATE_FOUND;
            break;
        }
        case ConversationKind::DROP_OFF: {
            accompanyOrder.phase = AccompanyPhase::CONVERSATE_DROPOFF;
            break;
        }
        case ConversationKind::ASK_DIRECTIONS: {
            accompanyOrder.phase = AccompanyPhase::CONVERSATE_ASK;
            break;
        }
    }
}

void AccompanyOrderPlugin::onStartDriveEvent(ISimContext& ctx, IOrder& order) {
    // not implemented
};

void AccompanyOrderPlugin::onStopDriveEvent(ISimContext& ctx, IOrder& order) {
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

OrderPtr AccompanyOrderPlugin::fromJson(const nlohmann::json& j) const {
    auto o = std::make_shared<AccompanyOrder>();
    o->id          = j.at("id");
    o->type        = "accompany";
    o->deadline    = j.at("appointmentTime");
    o->description = j.value("description", "");
    o->personName  = j.at("personName");
    o->roomName    = j.at("roomName");
    o->execution   = ExecutionMode::SCHEDULED;
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

int AccompanyOrderPlugin::planDispatchTime(const IOrder& order, const Scheduler& s, const std::string& startPos) const {
    const auto& mission = static_cast<const AccompanyOrder&>(order);
    return *mission.deadline - s.timeBuffer();
}

bool AccompanyOrderPlugin::isFeasible(const IOrder& order, const ISimContext& context) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);

    const auto robotLocation     = context.getRobot()->getLocation();
    const auto person            = context.getPersonByName(a.personName);
    const double missionDuration = meetingViaWorkplace(context.getScheduler(), person->workplace, robotLocation, a.roomName).total();

    const int slack = static_cast<int>(std::floor(order.deadline.value() - missionDuration - context.getTime()));

    if (slack >= 0) {
        DES_LOG_DEBUG("des.plugin.accompany", "Mission %u is feasible", order.id);
        return true;
    }
    DES_LOG_DEBUG("des.plugin.accompany",
                 "Mission %u (%s -> %s) infeasible: deadline %d, optimistic mission %.0fs from %s, now %d → slack %ds",
                 order.id, a.personName.c_str(), a.roomName.c_str(),
                 *order.deadline, missionDuration, robotLocation.c_str(),
                 context.getTime(), slack);
    return false;
}

std::optional<std::string> AccompanyOrderPlugin::targetLocation(const IOrder& order) const {
    return static_cast<const AccompanyOrder&>(order).roomName;
}

std::string AccompanyOrderPlugin::outcomeDetail(const IOrder& order) const {
    switch (static_cast<const AccompanyOrder&>(order).abortReason) {
        case SearchAbortReason::OUTSIDE:                 return "person outside";
        case SearchAbortReason::IN_BUILDING_FINDABLE:    return "missed in building";
        case SearchAbortReason::IN_BUILDING_UNREACHABLE: return "unreachable room";
        default:                                         return {};
    }
}

namespace {

struct MissionLegs {
    double searchSec    = 0.0;
    double searchWh     = 0.0;
    double accompanySec = 0.0;
    double talkSec      = 0.0;
    double driveBackSec = 0.0;
};

MissionLegs missionLegs(const AccompanyOrder& a, const ISimContext& context, const std::string& startLocation) {
    const auto& cfg   = *context.getConfig();
    const auto& acfg  = accompanyConfig();
    const auto& sched = context.getScheduler();
    const auto person = context.getPersonByName(a.personName);

    MissionLegs legs;
    if (const auto plan = planPersonSearch(context, a, startLocation)) {
        legs.searchSec = plan->durationSec;
        legs.searchWh  = plan->energyWh;
    } else {
        legs.searchSec = sched.robotDriveTime(startLocation, person->workplace)
                       + sched.getScanTime(person->workplace);
        legs.searchWh  = legs.searchSec * cfg.energyConsumptionDrive / 3600.0;
    }
    // Where the person is actually found is unknown while estimating, so the
    // accompany leg is measured from the workplace.
    legs.accompanySec = sched.getDriveTime(person->workplace, a.roomName, acfg.accompanySpeed);
    legs.talkSec      = 2.0 * acfg.conversationDurationMean;
    legs.driveBackSec = sched.robotDriveTime(a.roomName, cfg.dockLocation);
    return legs;
}

}  // namespace

double AccompanyOrderPlugin::estimateMissionEnergy(const IOrder& order, const ISimContext& context, const std::string& startLocation) const {
    const auto& a    = static_cast<const AccompanyOrder&>(order);
    const auto& cfg  = *context.getConfig();
    const auto legs  = missionLegs(a, context, startLocation);

    const double accompanyWh = legs.accompanySec * cfg.energyConsumptionDrive / 3600.0;
    const double talkWh      = legs.talkSec * cfg.energyConsumptionBase / 3600.0;
    const double driveBackWh = legs.driveBackSec * cfg.energyConsumptionDrive / 3600.0;
    return legs.searchWh + accompanyWh + talkWh + driveBackWh;
}

double AccompanyOrderPlugin::estimateMissionDuration(const IOrder& order, const ISimContext& context, const std::string& startLocation) const {
    const auto& a   = static_cast<const AccompanyOrder&>(order);
    const auto legs = missionLegs(a, context, startLocation);
    return legs.searchSec + legs.accompanySec + legs.talkSec + legs.driveBackSec;
}

void AccompanyOrderPlugin::publishTimeline(const IOrder& order, int startTime, ITimelineSink& sink) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);
    sink.publishMeeting(
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

}  // namespace des
