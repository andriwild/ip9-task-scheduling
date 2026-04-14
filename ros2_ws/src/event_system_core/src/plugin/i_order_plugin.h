#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>
#include "i_order.h"


class Scheduler;
class ISimContext;

class IOrderPlugin {
public:
    virtual ~IOrderPlugin() = default;

    virtual std::string typeName() const = 0;
    virtual std::string subtreeId() const = 0;

    virtual void registeredNodes(BT::BehaviorTreeFactory& factory) = 0;
    virtual std::string subtreeXml() const = 0;
    virtual des::OrderPtr fromJson(const nlohmann::json& j) const = 0;
    virtual int planDispatchTime(const des::IOrder& order, const Scheduler& scheduler) const = 0;
    virtual bool isFeasible(const des::IOrder& order, const ISimContext& context) = 0;
};
