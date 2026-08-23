#include "clean_plugin.h"

#include <cmath>
#include <memory>

#include "bt_nodes/clean.h"
#include "sim/scheduler.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include <algorithm>
#include "clean_order.h"
#include "states.h"
#include "util/types.h"

namespace des {

constexpr double kReferenceArea = 100.0;

void CleanPlugin::registeredNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<CleanIsAtTargetLocation>("CleanIsAtTargetLocation");
    factory.registerNodeType<CleanGoToLocation>("CleanGoToLocation");
    factory.registerNodeType<ExecuteClean>("ExecuteClean");
}

void CleanPlugin::onMissionStart(ISimContext& ctx, IOrder& /*order*/) {
    ctx.changeRobotState(std::make_unique<CleanState>());
}

void CleanPlugin::onMissionEnd(ISimContext& ctx, IOrder& order) {
    if (order.state == COMPLETED) {
        const auto& o = static_cast<const CleanOrder&>(order);
        ctx.recordServiced(o.roomName, kTypeName, ctx.getTime());
    }
    ctx.changeRobotState(std::make_unique<IdleState>());
}

double CleanPlugin::estimateReward(const IOrder& order, const EstimationView& view) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    const double areaUtility = std::min(1.0, view.world.room(o.roomName).m_area.value_or(1.0) / kReferenceArea);
    const double interval = o.cleaningInterval.value_or(m_config.cleaningInterval);

    double urgency = 1.0;
    const auto last = view.world.lastServiced(o.roomName, kTypeName);
    if (last.has_value() && interval > 0.0) {
        urgency = std::clamp((view.clock.getTime() - last.value()) / interval, 0.0, 1.0);
    }
    return m_config.rewardWeight * areaUtility * urgency;
}

void CleanPlugin::onStartDriveEvent(ISimContext& /*ctx*/, IOrder& /*order*/) {}

void CleanPlugin::onStopDriveEvent(ISimContext& /*ctx*/, IOrder& /*order*/) {}

OrderPtr CleanPlugin::fromJson(const nlohmann::json& j) const {
    auto o = std::make_shared<CleanOrder>();
    o->id          = j.at("id");
    o->type        = "clean";
    o->deadline    = j.contains("appointmentTime") ? std::optional<int>(j.at("appointmentTime").get<int>()) : std::nullopt;
    o->description = j.value("description", "Clean");
    o->roomName    = j.at("roomName");
    if (j.contains("cleaning_interval")) {
        o->cleaningInterval = j.at("cleaning_interval").get<double>();
    }
    o->execution   = ExecutionMode::BACKGROUND;
    return o;
}

int CleanPlugin::planDispatchTime(const IOrder& order, const Scheduler& s, const std::string& startPos) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    if (!o.deadline.has_value()) {
        return o.dispatchTime;
    }
    const double driveTime = s.robotDriveTime(startPos, o.roomName);
    return *o.deadline - static_cast<int>(driveTime) - s.timeBuffer();
}

bool CleanPlugin::isFeasible(const IOrder& order, const ISimContext& context) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    if (!o.deadline.has_value()) {
        return true;
    }
    const double driveTime = context.getScheduler().robotDriveTime(context.getRobot()->getLocation(), o.roomName);
    const int slack = static_cast<int>(*o.deadline - driveTime - context.getTime());
    if (slack < 0) {
        DES_LOG_DEBUG("des.plugin.clean",
                     "Mission %d infeasible: deadline %d, driveTime %.0fs from %s, now %d → slack %ds",
                     o.id, *o.deadline, driveTime, context.getRobot()->getLocation().c_str(),
                     context.getTime(), slack);
        return false;
    }
    return true;
}

std::optional<std::string> CleanPlugin::targetLocation(const IOrder& order) const {
    return static_cast<const CleanOrder&>(order).roomName;
}

// Cleaning is modelled as a lawnmower pass over the room area.
double cleanDurationSeconds(const double roomArea, const double robotSpeed) {
    const double cleaningArea = cleanConfig().cleaningArea;
    const double cleaningSide = std::sqrt(cleaningArea);
    const double steps        = (roomArea / cleaningArea) + 1;
    return steps * (2.0 * cleaningSide / robotSpeed);
}

double CleanPlugin::estimateServiceDuration(const IOrder& order, const EstimationView& view) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    return cleanDurationSeconds(view.world.room(o.roomName).m_area.value_or(1.0), view.cfg.robotSpeed);
}

double CleanPlugin::estimateServiceEnergy(const IOrder& order, const EstimationView& view) const {
    return estimateServiceDuration(order, view) * m_config.cleaningPower / 3600.0;
}

void CleanPlugin::publishTimeline(const IOrder& order, int startTime, ITimelineSink& sink) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    sink.publishMeeting(
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

}  // namespace des
