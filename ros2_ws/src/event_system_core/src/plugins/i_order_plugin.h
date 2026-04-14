#pragma once

#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include "i_order.h"

namespace BT { class BehaviorTreeFactory; }
class Scheduler;
class ISimContext;
class RosObserver;

class IOrderPlugin {
public:
    virtual ~IOrderPlugin() = default;

    virtual std::string typeName() const = 0;
    virtual std::vector<std::string> subtreeIds() const = 0;

    virtual void registeredNodes(BT::BehaviorTreeFactory& factory) = 0;
    virtual std::string subtreeXml() const = 0;
    virtual des::OrderPtr fromJson(const nlohmann::json& j) const = 0;
    virtual int planDispatchTime(const des::IOrder& order, const Scheduler& scheduler, const std::string& startPos) const = 0;
    virtual bool isFeasible(const des::IOrder& order, const ISimContext& context) const = 0;
    virtual void publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const = 0;
};
