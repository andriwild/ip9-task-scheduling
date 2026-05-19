#include "clean_plugin.h"

#include <memory>

#include "bt_nodes/clean.h"
#include "sim/scheduler.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "observer/ros.h"
#include "clean_order.h"
#include "util/types.h"

void CleanPlugin::registeredNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<CleanIsAtTargetLocation>("CleanIsAtTargetLocation");
    factory.registerNodeType<CleanGoToLocation>("CleanGoToLocation");
    factory.registerNodeType<ExecuteClean>("ExecuteClean");
}

void CleanPlugin::onMissionStart(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

void CleanPlugin::onMissionEnd(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

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
    return *o.deadline - driveTime >= context.getTime();
}

void CleanPlugin::publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const {
    const auto& o = static_cast<const CleanOrder&>(order);
    observer.publishMeeting(
        o.id,
        startTime,
        o.deadline.value_or(startTime),
        static_cast<int>(o.state),
        "Clean",
        o.description,
        static_cast<int>(o.execution));
}
