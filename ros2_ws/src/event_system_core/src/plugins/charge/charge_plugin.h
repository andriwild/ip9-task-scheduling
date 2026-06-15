#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "./charge_subtree.h"
#include "./charge_order.h"

class Scheduler;
class ISimContext;
class RosObserver;

class ChargePlugin : public IOrderPlugin {
public:
    static constexpr auto kTypeName = kChargeOrderType;

    explicit ChargePlugin() = default;

    std::string typeName() const override { return kTypeName; }
    std::string rootSubtreeId() const override { return "BackgroundChargeRoutine"; }
    des::ExecutionMode executionMode() const override { return des::ExecutionMode::BACKGROUND; }

    void onMissionStart(ISimContext& ctx, des::IOrder& order) override;
    void onMissionEnd(ISimContext& ctx, des::IOrder& order) override;
    void onStartDriveEvent(ISimContext& ctx, des::IOrder& order) override;
    void onStopDriveEvent(ISimContext& ctx, des::IOrder& order) override;

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return CHARGE_SUBTREE_XML; }
    des::OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const des::IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const des::IOrder& order, const ISimContext& context) const override;
    std::optional<std::string> targetLocation(const des::IOrder& order) const override;
    void publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const override;
};
