#include "clean_plugin.h"

#include <cmath>
#include <memory>

#include "bt_nodes/clean.h"
#include "sim/scheduler.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "observer/ros.h"
#include "clean_order.h"
#include "states.h"
#include "util/types.h"

void CleanPlugin::registeredNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<CleanIsAtTargetLocation>("CleanIsAtTargetLocation");
    factory.registerNodeType<CleanGoToLocation>("CleanGoToLocation");
    factory.registerNodeType<ExecuteClean>("ExecuteClean");
}

void CleanPlugin::onMissionStart(ISimContext& ctx, des::IOrder& /*order*/) {
    ctx.changeRobotState(std::make_unique<CleanState>());
}

void CleanPlugin::onMissionEnd(ISimContext& ctx, des::IOrder& /*order*/) {
    ctx.changeRobotState(std::make_unique<IdleState>());
}

void CleanPlugin::onStartDriveEvent(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

void CleanPlugin::onStopDriveEvent(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

des::OrderPtr CleanPlugin::fromJson(const nlohmann::json& j) const {
    auto o = std::make_shared<CleanOrder>();
    o->id          = j.at("id");
    o->type        = "clean";
    o->deadline    = j.contains("appointmentTime") ? std::optional<int>(j.at("appointmentTime").get<int>()) : std::nullopt;
    o->description = j.value("description", "Clean");
    o->roomName    = j.at("roomName");
    o->execution   = des::ExecutionMode::BACKGROUND;
    return o;
}

int CleanPlugin::planDispatchTime(const des::IOrder& order, const Scheduler& s, const std::string& startPos) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    if (!o.deadline.has_value()) {
        return o.dispatchTime;
    }
    const double driveTime = s.robotDriveTime(startPos, o.roomName);
    return *o.deadline - static_cast<int>(driveTime) - s.timeBuffer();
}

bool CleanPlugin::isFeasible(const des::IOrder& order, const ISimContext& context) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    if (!o.deadline.has_value()) {
        return true;
    }
    const double driveTime = context.getScheduler().robotDriveTime(
        context.getRobot()->getLocation(), o.roomName);
    const int slack = static_cast<int>(*o.deadline - driveTime - context.getTime());
    if (slack < 0) {
        DES_LOG_WARN(rclcpp::get_logger("des.plugin.clean"),
                     "Mission %d infeasible: deadline %d, driveTime %.0fs from %s, now %d → slack %ds",
                     o.id, *o.deadline, driveTime, context.getRobot()->getLocation().c_str(),
                     context.getTime(), slack);
        return false;
    }
    return true;
}

namespace {
struct CleanTimings { double driveOut; double cleanTime; double driveBack; };

CleanTimings cleanTimings(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) {
    const auto& o     = static_cast<const CleanOrder&>(order);
    const auto& sched = context.getScheduler();
    const auto& cfg   = *context.getConfig();

    const double driveOut  = sched.robotDriveTime(startLocation, o.roomName);
    const double driveBack = sched.robotDriveTime(o.roomName, cfg.dockLocation);

    const double roomArea       = context.getSearchArea(o.roomName);
    const double broomFootprint = cleanConfig().cleaningArea;
    const double broomSide      = std::sqrt(broomFootprint);
    const double steps          = (roomArea / broomFootprint) + 1;
    const double cleanTime      = steps * (2.0 * broomSide / cfg.robotSpeed);
    return {driveOut, cleanTime, driveBack};
}
}

double CleanPlugin::estimateMissionEnergy(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) const {
    const auto t   = cleanTimings(order, context, startLocation);
    const auto& cfg = *context.getConfig();
    return ((t.driveOut + t.driveBack) * cfg.energyConsumptionDrive + t.cleanTime * cfg.energyConsumptionBase) / 3600.0;
}

double CleanPlugin::estimateMissionDuration(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) const {
    const auto t = cleanTimings(order, context, startLocation);
    return t.driveOut + t.cleanTime + t.driveBack;
}

void CleanPlugin::publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    observer.publishMeeting(
        o.id,
        startTime,
        o.deadline.value_or(startTime),
        static_cast<int>(o.state),
        kTypeName,
        "",
        o.roomName,
        o.description,
        static_cast<int>(o.execution));
}
