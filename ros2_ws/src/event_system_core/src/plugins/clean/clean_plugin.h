#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "./clean_subtree.h"


class Scheduler;
class ISimContext;
class RosObserver;

class CleanPlugin: public IOrderPlugin {
public:
    explicit CleanPlugin() = default;

    std::string typeName() const override { return "clean"; }
    std::string rootSubtreeId() const override { return "CleanRoutine"; }
    des::ExecutionMode executionMode() const override { return des::ExecutionMode::BACKGROUND; }

    void onMissionStart(ISimContext& ctx, des::IOrder& order) override;
    void onMissionEnd(ISimContext& ctx, des::IOrder& order) override;
    virtual void onStartDriveEvent(ISimContext& ctx, des::IOrder& order) override;
    virtual void onStopDriveEvent(ISimContext& ctx, des::IOrder& order) override;

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return CLEAN_SUBTREE_XML; }
    des::OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const des::IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const des::IOrder& order, const ISimContext& context) const override;
    void publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const override;
};
