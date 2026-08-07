#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "./charge_subtree.h"
#include "./charge_order.h"

namespace des {

class Scheduler;
class ISimContext;
class RosObserver;
class ChargePlugin : public IOrderPlugin {
public:
    static constexpr auto kTypeName = kChargeOrderType;

    explicit ChargePlugin() = default;

    std::string typeName() const override { return kTypeName; }
    std::string rootSubtreeId() const override { return "BackgroundChargeRoutine"; }
    ExecutionMode executionMode() const override { return ExecutionMode::BACKGROUND; }

    void onMissionStart(ISimContext& ctx, IOrder& order) override;
    void onMissionEnd(ISimContext& ctx, IOrder& order) override;
    void onStartDriveEvent(ISimContext& ctx, IOrder& order) override;
    void onStopDriveEvent(ISimContext& ctx, IOrder& order) override;

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return CHARGE_SUBTREE_XML; }
    OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const IOrder& order, const ISimContext& context) const override;
    std::optional<std::string> targetLocation(const IOrder& order) const override;
    void publishTimeline(const IOrder& order, int startTime, RosObserver& observer) const override;
};

}  // namespace des
