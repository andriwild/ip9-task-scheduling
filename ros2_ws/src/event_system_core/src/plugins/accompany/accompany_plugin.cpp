#include "accompany_plugin.h"
#include "../../util/log.h"

#include <memory>

#include "bt_nodes/search.h"
#include "bt_nodes/accompany.h"
#include "bt_nodes/conversation.h"
#include "events/appointment_end_event.h"
#include "sim/scheduler.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
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
    if (ctx.hasEmployee(personName)) {
        auto person = ctx.getPersonByName(personName);
        int endTime = ctx.getTime() + static_cast<int>(accompanyConfig().appointmentDuration);
        ctx.pushEvent(std::make_shared<AppointmentEndEvent>(endTime, person));
    }
}

void AccompanyOrderPlugin::onMissionStart(ISimContext& ctx, des::IOrder& order) {
    auto accompanyOrder = static_cast<AccompanyOrder&>(order);
    auto locations = ctx.getPersonByName(accompanyOrder.personName)->roomLabels;
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
// Optimistic estimate: assume the person is in their *first* known room.
// Used for feasibility checks where we don't want to over-reject missions
// just because the search could in principle take longer.
double optimisticMeeting(const Scheduler& sched, const std::string& personName, const std::string& startPos, const std::string& goalPos) {
    const auto& rooms          = sched.employeeRooms(personName);
    const auto employeeLoc     = rooms.front();
    const double accompanySpd  = accompanyConfig().accompanySpeed;
    const double searchTime    = sched.robotDriveTime(startPos, employeeLoc);
    const double scanTime      = sched.getScanTime(employeeLoc);
    const double accompanyTime = sched.getDriveTime(employeeLoc, goalPos, accompanySpd);
    return searchTime + accompanyTime + scanTime;
}

// Pessimistic estimate: assume the search visits AND scans every known room
// before finding the person in the last one. Used for dispatch scheduling so
// we leave enough lead time.
double pessimisticMeeting(const Scheduler& sched, const std::string& personName, const std::string& startPos, const std::string& goalPos) {
    const auto& rooms         = sched.employeeRooms(personName);
    const double accompanySpd = accompanyConfig().accompanySpeed;
    double searchTime = 0.0;
    std::string currentPos = startPos;
    for (const auto& location : rooms) {
        searchTime += sched.robotDriveTime(currentPos, location);
        searchTime += sched.getScanTime(location);
        currentPos = location;
    }
    const double accompanyTime = sched.getDriveTime(currentPos, goalPos, accompanySpd);
    return searchTime + accompanyTime;
}
}

int AccompanyOrderPlugin::planDispatchTime(const des::IOrder& order, const Scheduler& s, const std::string& startPos) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);
    const double driveTime = pessimisticMeeting(s, a.personName, startPos, a.roomName);
    return *a.deadline - static_cast<int>(driveTime) - s.timeBuffer();
}

bool AccompanyOrderPlugin::isFeasible(const des::IOrder& order, const ISimContext& context) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);

    const auto robotLocation = context.getRobot()->getLocation();
    const double missionDuration = optimisticMeeting(context.getScheduler(), a.personName, robotLocation, a.roomName);
    const int slack = static_cast<int>(order.deadline.value() - missionDuration - context.getTime());
    if (slack >= 0) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.accompany"), "Mission %u is feasible", order.id);
        return true;
    }
    DES_LOG_WARN(rclcpp::get_logger("des.plugin.accompany"),
                 "Mission %u (%s -> %s) infeasible: deadline %d, optimistic mission %.0fs from %s, now %d → slack %ds",
                 order.id, a.personName.c_str(), a.roomName.c_str(),
                 *order.deadline, missionDuration, robotLocation.c_str(),
                 context.getTime(), slack);
    return false;
}

namespace {
struct AccompanyTimings { double meeting; double appointment; double driveBack; };

AccompanyTimings accompanyTimings(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) {
    const auto& a     = static_cast<const AccompanyOrder&>(order);
    const auto& sched = context.getScheduler();
    const auto& cfg   = *context.getConfig();
    return {
        pessimisticMeeting(sched, a.personName, startLocation, a.roomName),
        accompanyConfig().appointmentDuration,
        sched.robotDriveTime(a.roomName, cfg.dockLocation)
    };
}
}

double AccompanyOrderPlugin::estimateMissionEnergy(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) const {
    const auto t   = accompanyTimings(order, context, startLocation);
    const auto& cfg = *context.getConfig();
    // Worst-case meeting (search + accompany) treated as driving for upper bound.
    return (t.meeting * cfg.energyConsumptionDrive
          + t.appointment * cfg.energyConsumptionBase
          + t.driveBack   * cfg.energyConsumptionDrive) / 3600.0;
}

double AccompanyOrderPlugin::estimateMissionDuration(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) const {
    const auto t = accompanyTimings(order, context, startLocation);
    return t.meeting + t.appointment + t.driveBack;
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
