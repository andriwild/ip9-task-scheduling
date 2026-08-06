#include "clean_plugin.h"

#include <cmath>
#include <memory>

#include "bt_nodes/clean.h"
#include "sim/scheduler.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "observer/ros.h"
#include <algorithm>
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

void CleanPlugin::onMissionEnd(ISimContext& ctx, des::IOrder& order) {
    if (order.state == des::COMPLETED) {
        const auto& o = static_cast<const CleanOrder&>(order);
        ctx.recordServiced(o.roomName, kTypeName, ctx.getTime());
    }
    ctx.changeRobotState(std::make_unique<IdleState>());
}

double CleanPlugin::estimateReward(const des::IOrder& order, const EstimationView& view) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    const double areaUtility = std::min(1.0, view.world.room(o.roomName).m_area.value_or(1.0) / 100.0);
    const double interval = o.cleaningInterval.value_or(m_config.cleaningInterval);

    double urgency = 1.0;
    const auto last = view.world.lastServiced(o.roomName, kTypeName);
    if (last.has_value() && interval > 0.0) {
        urgency = std::clamp((view.clock.getTime() - last.value()) / interval, 0.0, 1.0);
    }
    return m_config.rewardWeight * areaUtility * urgency;
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
    if (j.contains("cleaning_interval")) {
        o->cleaningInterval = j.at("cleaning_interval").get<double>();
    }
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
    const double driveTime = context.getScheduler().robotDriveTime(context.getRobot()->getLocation(), o.roomName);
    const int slack = static_cast<int>(*o.deadline - driveTime - context.getTime());
    if (slack < 0) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.plugin.clean"),
                     "Mission %d infeasible: deadline %d, driveTime %.0fs from %s, now %d → slack %ds",
                     o.id, *o.deadline, driveTime, context.getRobot()->getLocation().c_str(),
                     context.getTime(), slack);
        return false;
    }
    return true;
}

std::optional<std::string> CleanPlugin::targetLocation(const des::IOrder& order) const {
    return static_cast<const CleanOrder&>(order).roomName;
}

double CleanPlugin::estimateServiceDuration(const des::IOrder& order, const EstimationView& view) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    const auto& cfg = view.cfg;

    const double roomArea     = view.world.room(o.roomName).m_area.value_or(1.0);
    const double cleaningSide = std::sqrt(m_config.cleaningArea);
    const double steps        = (roomArea / m_config.cleaningArea) + 1;
    return steps * (2.0 * cleaningSide / cfg.robotSpeed);
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
