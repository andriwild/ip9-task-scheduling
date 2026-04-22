#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "./data_acquisition_subtree.h"


class Scheduler;
class ISimContext;
class RosObserver;

class DataAcquisition: public IOrderPlugin {
public:
    explicit DataAcquisition() = default;

    std::string typeName() const override { return "data_acquisition"; }
    std::string rootSubtreeId() const override { return "DataAcquisitionRoutine"; }

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return DATA_ACQUISITION_SUBTREE_XML; }
    des::OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const des::IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const des::IOrder& order, const ISimContext& context) const override;
    void publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const override;
};
