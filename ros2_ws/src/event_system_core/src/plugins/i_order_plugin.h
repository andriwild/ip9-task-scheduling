#pragma once

#include <optional>
#include <string>
#include <nlohmann/json.hpp>

#include "model/order.h"
#include "../util/types.h"
#include "engine/contracts/estimation_view.h"
#include "engine/contracts/i_sim_context.h"
#include "engine/contracts/i_timeline_sink.h"
#include "model/sim_config.h"

namespace BT { class BehaviorTreeFactory; }

namespace des {


class Scheduler;
class IOrderPlugin {
public:
    virtual ~IOrderPlugin() = default;

    virtual std::string typeName() const = 0;
    virtual std::string rootSubtreeId() const = 0;
    virtual ExecutionMode executionMode() const = 0;

    // hooks
    virtual void onMissionStart(ISimContext& ctx, IOrder& order)    = 0;
    virtual void onMissionEnd(ISimContext& ctx, IOrder& order)      = 0;

    virtual void onMissionResume(ISimContext& ctx, IOrder& order) {
        onMissionStart(ctx, order);
    }

    virtual void onStartDriveEvent(ISimContext& ctx, IOrder& order) = 0;
    virtual void onStopDriveEvent(ISimContext& ctx, IOrder& order)  = 0;

    virtual void onConversationStart(ISimContext& /*ctx*/, IOrder& /*order*/, ConversationKind /*kind*/) {}

    virtual void registeredNodes(BT::BehaviorTreeFactory& factory) = 0;
    virtual std::string subtreeXml() const = 0;
    virtual OrderPtr fromJson(const nlohmann::json& j) const = 0;

    virtual void loadConfig(const nlohmann::json& /*pluginCfg*/) {}
    virtual nlohmann::json saveConfig() const { return nlohmann::json::object(); }
    virtual int planDispatchTime(const IOrder& order, const Scheduler& scheduler, const std::string& startPos) const = 0;
    virtual bool isFeasible(const IOrder& order, const ISimContext& context) const = 0;

    virtual std::string outcomeDetail(const IOrder& /*order*/) const {
        return {};
    }

    // Waypoint the mission is executed at, used for route planning.
    // nullopt = mission has no single target and cannot be routed.

    virtual std::optional<std::string> targetLocation(const IOrder& /*order*/) const {
        return std::nullopt;
    }

    // OP reward for visiting this mission's location. Default: the room area.
    virtual double estimateReward(const IOrder& order, const EstimationView& view) const {
        const auto target = targetLocation(order);
        return target ? view.world.room(*target).m_area.value_or(1.0) : 0.0;
    }

    // On-site execution time in seconds — no drive legs.
    virtual double estimateServiceDuration(const IOrder& /*order*/,
                                           const EstimationView& /*view*/) const {
        return 0.0;
    }

    // On-site execution energy in Wh — no drive legs.
    virtual double estimateServiceEnergy(const IOrder& order,
                                         const EstimationView& view) const {
        return estimateServiceDuration(order, view) * view.cfg.energyConsumptionBase / 3600.0;
    }

    // Round-trip energy estimate in Wh for executing the mission from
    // `startLocation` and returning the robot to the dock
    virtual double estimateMissionEnergy(const IOrder& /*order*/,
                                         const ISimContext& /*context*/,
                                         const std::string& /*startLocation*/) const {
        return 0.0;
    }

    // Round-trip duration in seconds: drive to mission location + execute +
    // drive back to dock. 
    virtual double estimateMissionDuration(const IOrder& /*order*/,
                                           const ISimContext& /*context*/,
                                           const std::string& /*startLocation*/) const {
        return 0.0;
    }

    virtual void publishTimeline(const IOrder& order, int startTime, ITimelineSink& sink) const = 0;
};

}  // namespace des
