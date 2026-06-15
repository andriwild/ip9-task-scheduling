#include "charge_plugin.h"

#include <memory>

#include "bt_nodes/charge_mission.h"
#include "charge_order.h"
#include "model/i_sim_context.h"
#include "model/robot_state.h"

void ChargePlugin::registeredNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ChargeMissionIsAtDock>("ChargeMissionIsAtDock");
    factory.registerNodeType<ChargeMissionGoToDock>("ChargeMissionGoToDock");
    factory.registerNodeType<ExecuteChargeMission>("ExecuteChargeMission");
}

void ChargePlugin::onMissionStart(ISimContext& ctx, des::IOrder& /*order*/) {
    ctx.changeRobotState(std::make_unique<ChargeState>());
}

void ChargePlugin::onMissionEnd(ISimContext& ctx, des::IOrder& /*order*/) {
    ctx.changeRobotState(std::make_unique<IdleState>());
}

void ChargePlugin::onStartDriveEvent(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

void ChargePlugin::onStopDriveEvent(ISimContext& /*ctx*/, des::IOrder& /*order*/) {}

des::OrderPtr ChargePlugin::fromJson(const nlohmann::json& /*j*/) const {
    auto o       = std::make_shared<ChargeOrder>();
    o->type      = kChargeOrderType;
    o->execution = des::ExecutionMode::BACKGROUND;
    return o;
}

int ChargePlugin::planDispatchTime(const des::IOrder& order, const Scheduler& /*scheduler*/, const std::string& /*startPos*/) const {
    return order.dispatchTime;
}

bool ChargePlugin::isFeasible(const des::IOrder& /*order*/, const ISimContext& /*context*/) const {
    return true;
}

std::optional<std::string> ChargePlugin::targetLocation(const des::IOrder& order) const {
    return static_cast<const ChargeOrder&>(order).dockLocation;
}

void ChargePlugin::publishTimeline(const des::IOrder& /*order*/, int /*startTime*/, RosObserver& /*observer*/) const {}
