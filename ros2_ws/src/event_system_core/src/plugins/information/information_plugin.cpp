#include "information_plugin.h"

#include <memory>

#include "bt_nodes/information.h"
#include "observer/ros.h"
#include "information_order.h"
#include "states.h"

void InformationPlugin::registeredNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ExecuteInformation>("ExecuteInformation");
}

void InformationPlugin::onMissionStart(ISimContext& ctx, des::IOrder& /*order*/) {
    ctx.changeRobotState(std::make_unique<InformationState>());
}

// pre-interrupt state is restored by popInterrupt
void InformationPlugin::onMissionEnd(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}
void InformationPlugin::onStartDriveEvent(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}
void InformationPlugin::onStopDriveEvent(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

des::OrderPtr InformationPlugin::fromJson(const nlohmann::json& j) const {
    auto o = std::make_shared<InformationOrder>();
    o->id          = j.at("id");
    o->type        = "information";
    o->description = j.value("description", "Information");
    o->execution   = des::ExecutionMode::INTERRUPT;
    return o;
}

int InformationPlugin::planDispatchTime(const des::IOrder& order, const Scheduler& /*s*/, const std::string& /*startPos*/) const {
    return order.dispatchTime;
}

bool InformationPlugin::isFeasible(const des::IOrder& /*order*/, const ISimContext& /*context*/) const {
    return true;
}

double InformationPlugin::estimateMissionDuration(const des::IOrder& /*order*/, const ISimContext& /*context*/, const std::string& /*startLocation*/) const {
    return informationConfig().informationDuration;
}

void InformationPlugin::publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const {
    observer.publishMeeting(
        order.id,
        startTime,
        startTime,
        static_cast<int>(order.state),
        kTypeName,
        "",
        "",
        order.description,
        static_cast<int>(order.execution));
}
