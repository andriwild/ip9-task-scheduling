#pragma once
#include "util/constants.h"

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "../order_registry.h"
#include "./clean_subtree.h"


namespace des {

class Scheduler;
class ISimContext;
class RosObserver;
struct CleanConfig {
    double cleaningArea = 0.09;
    double rewardWeight = 0.23;
    double cleaningInterval = SECONDS_PER_DAY;
};

class CleanPlugin: public IOrderPlugin {
    CleanConfig m_config;
public:
    static constexpr auto kTypeName = "clean";

    explicit CleanPlugin() = default;

    std::string typeName() const override { return kTypeName; }
    std::string rootSubtreeId() const override { return "CleanRoutine"; }
    ExecutionMode executionMode() const override { return ExecutionMode::BACKGROUND; }

    void onMissionStart(ISimContext& ctx, IOrder& order) override;
    void onMissionEnd(ISimContext& ctx, IOrder& order) override;
    virtual void onStartDriveEvent(ISimContext& ctx, IOrder& order) override;
    virtual void onStopDriveEvent(ISimContext& ctx, IOrder& order) override;

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return CLEAN_SUBTREE_XML; }
    OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const IOrder& order, const ISimContext& context) const override;
    std::optional<std::string> targetLocation(const IOrder& order) const override;
    double estimateServiceDuration(const IOrder& order, const EstimationView& view) const override;
    double estimateReward(const IOrder& order, const EstimationView& view) const override;
    void publishTimeline(const IOrder& order, int startTime, RosObserver& observer) const override;

    const CleanConfig& config() const { return m_config; }

    void loadConfig(const nlohmann::json& j) override {
        m_config.cleaningArea     = j.value("cleaning_area", m_config.cleaningArea);
        m_config.rewardWeight     = j.value("reward_weight", m_config.rewardWeight);
        m_config.cleaningInterval = j.value("cleaning_interval", m_config.cleaningInterval);
    }
    nlohmann::json saveConfig() const override {
        return {
            {"cleaning_area", m_config.cleaningArea},
            {"reward_weight", m_config.rewardWeight},
            {"cleaning_interval", m_config.cleaningInterval},
        };
    }
};

inline const CleanConfig& cleanConfig() {
    return static_cast<const CleanPlugin&>(
        OrderRegistry::instance().get(CleanPlugin::kTypeName)).config();
}

}  // namespace des
