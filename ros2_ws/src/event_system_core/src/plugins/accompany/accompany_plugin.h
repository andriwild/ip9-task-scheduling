#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "accompany_subtree.h"

class Scheduler;
class ISimContext;
class RosObserver;

class AccompanyOrderPlugin : public IOrderPlugin {
public:
    explicit AccompanyOrderPlugin() = default;

    std::string typeName() const override { return "accompany"; }
    std::string rootSubtreeId() const override { return "AccompanyRoutine"; }

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return ACCOMPANY_SUBTREE_XML; }
    des::OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const des::IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const des::IOrder& order, const ISimContext& context) const override;
    void publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const override;
};
