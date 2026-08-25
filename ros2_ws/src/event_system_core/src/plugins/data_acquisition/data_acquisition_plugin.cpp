#include "data_acquisition_plugin.h"

#include <algorithm>
#include <memory>

#include "bt_nodes/acquisition.h"
#include "sim/scheduler.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "data_acquisition_order.h"
#include "states.h"

namespace des {

void DataAcquisition::registeredNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<IsAtTargetLocation>("IsAtTargetLocation");
    factory.registerNodeType<GoToLocation>("GoToLocation");
    factory.registerNodeType<ExecuteAcquisition>("ExecuteAcquisition");
}

void DataAcquisition::onMissionStart(ISimContext& ctx, IOrder& /*order*/) {
    ctx.changeRobotState(std::make_unique<AcquireState>());
}

void DataAcquisition::onMissionEnd(ISimContext& ctx, IOrder& order) {
    if (order.state == COMPLETED) {
        const auto& o = static_cast<const DataAcquisitionOrder&>(order);
        ctx.recordServiced(o.roomName, kTypeName, ctx.getTime());
    }
    ctx.changeRobotState(std::make_unique<IdleState>());
}

double DataAcquisition::estimateReward(const IOrder& order, const EstimationView& view) const {
    const auto& o = static_cast<const DataAcquisitionOrder&>(order);
    const double interval = o.acquisitionInterval.value_or(m_config.acquisitionInterval);

    double due = 1.0;
    const auto last = view.world.lastServiced(o.roomName, kTypeName);
    if (last.has_value() && interval > 0.0) {
        due = std::max(0.0, (view.clock.getTime() - last.value()) / interval);
    }
    return m_config.missionValue * due;
}

void DataAcquisition::onStartDriveEvent(ISimContext& /*ctx*/, IOrder& /*order*/) {}

void DataAcquisition::onStopDriveEvent(ISimContext& /*ctx*/, IOrder& /*order*/) {}

OrderPtr DataAcquisition::fromJson(const nlohmann::json& j) const {
    auto o = std::make_shared<DataAcquisitionOrder>();
    o->id          = j.at("id");
    o->type        = "data_acquisition";
    o->deadline    = j.contains("appointmentTime") ? std::optional<int>(j.at("appointmentTime").get<int>()) : std::nullopt;
    o->description = j.value("description", "");
    o->roomName    = j.at("roomName");
    if (j.contains("data_acquisition_interval")) {
        o->acquisitionInterval = j.at("data_acquisition_interval").get<double>();
    }
    o->execution   = ExecutionMode::BACKGROUND;
    return o;
}

int DataAcquisition::planDispatchTime(const IOrder& order, const Scheduler& s, const std::string& startPos) const {
    const auto& o = static_cast<const DataAcquisitionOrder&>(order);
    if (!o.deadline.has_value()) {
        return o.dispatchTime;
    }
    const double driveTime = s.robotDriveTime(startPos, o.roomName);
    return *o.deadline - static_cast<int>(driveTime) - s.timeBuffer();
}

bool DataAcquisition::isFeasible(const IOrder& order, const ISimContext& context) const {
    const auto& o = static_cast<const DataAcquisitionOrder&>(order);
    if (!o.deadline.has_value()) {
        return true;
    }
    const double driveTime = context.getScheduler().robotDriveTime(
        context.getRobot()->getLocation(), o.roomName);
    const int slack = static_cast<int>(*o.deadline - driveTime - context.getTime());
    if (slack < 0) {
        DES_LOG_DEBUG("des.plugin.data_acquisition",
                     "Mission %d infeasible: deadline %d, driveTime %.0fs from %s, now %d → slack %ds",
                     o.id, *o.deadline, driveTime, context.getRobot()->getLocation().c_str(),
                     context.getTime(), slack);
        return false;
    }
    return true;
}

std::optional<std::string> DataAcquisition::targetLocation(const IOrder& order) const {
    return static_cast<const DataAcquisitionOrder&>(order).roomName;
}

double DataAcquisition::estimateServiceDuration(const IOrder& /*order*/, const EstimationView& /*view*/) const {
    return m_config.dataAcquisitionDuration;
}

void DataAcquisition::publishTimeline(const IOrder& order, int startTime, ITimelineSink& sink) const {
    const auto& o = static_cast<const DataAcquisitionOrder&>(order);
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
