#include "data_acquisition_plugin.h"

#include <memory>

#include "bt_nodes/acquisition.h"
#include "sim/scheduler.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "observer/ros.h"
#include "data_acquisition_order.h"

void DataAcquisition::registeredNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<IsAtTargetLocation>("IsAtTargetLocation");
    factory.registerNodeType<GoToLocation>("GoToLocation");
    factory.registerNodeType<ExecuteAcquisition>("ExecuteAcquisition");
}

void DataAcquisition::onMissionStart(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

void DataAcquisition::onMissionEnd(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

void DataAcquisition::onStartDriveEvent(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

void DataAcquisition::onStopDriveEvent(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

des::OrderPtr DataAcquisition::fromJson(const nlohmann::json& j) const {
    auto o = std::make_shared<DataAcquisitionOrder>();
    o->id          = j.at("id");
    o->type        = "data_acquisition";
    o->deadline    = j.contains("appointmentTime") ? std::optional<int>(j.at("appointmentTime").get<int>()) : std::nullopt;
    o->description = j.value("description", "");
    o->roomName    = j.at("roomName");
    return o;
}

int DataAcquisition::planDispatchTime(const des::IOrder& order, const Scheduler& s, const std::string& startPos) const {
    const auto& o = static_cast<const DataAcquisitionOrder&>(order);
    if (!o.deadline.has_value()) {
        return o.dispatchTime;
    }
    const double driveTime = s.robotDriveTime(startPos, o.roomName);
    return *o.deadline - static_cast<int>(driveTime) - s.timeBuffer();
}

bool DataAcquisition::isFeasible(const des::IOrder& order, const ISimContext& context) const {
    const auto& o = static_cast<const DataAcquisitionOrder&>(order);
    if (!o.deadline.has_value()) {
        return true;
    }
    const double driveTime = context.getScheduler().robotDriveTime(
        context.getRobot()->getLocation(), o.roomName);
    return *o.deadline - driveTime >= context.getTime();
}

void DataAcquisition::publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const {
    const auto& o = static_cast<const DataAcquisitionOrder&>(order);
    observer.publishMeeting(
        o.id,
        startTime,
        o.deadline.value_or(startTime),
        static_cast<int>(o.state),
        "DataAcquisition",
        o.description);
}
