#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "../order_registry.h"
#include "./information_subtree.h"

namespace des {

class Scheduler;
class ISimContext;
class RosObserver;
struct InformationConfig {
    double informationDurationMin = 30.0;
    double informationDurationMax = 120.0;
};

class InformationPlugin : public IOrderPlugin {
    InformationConfig m_config;
public:
    static constexpr auto kTypeName = "information";

    explicit InformationPlugin() = default;

    std::string typeName() const override { return kTypeName; }
    std::string rootSubtreeId() const override { return "InformationRoutine"; }
    ExecutionMode executionMode() const override { return ExecutionMode::INTERRUPT; }

    void onMissionStart(ISimContext& ctx, IOrder& order) override;
    void onMissionEnd(ISimContext& ctx, IOrder& order) override;
    void onStartDriveEvent(ISimContext& ctx, IOrder& order) override;
    void onStopDriveEvent(ISimContext& ctx, IOrder& order) override;

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return INFORMATION_SUBTREE_XML; }
    OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const IOrder& order, const ISimContext& context) const override;
    double estimateMissionDuration(const IOrder& order, const ISimContext& context, const std::string& startLocation) const override;
    void publishTimeline(const IOrder& order, int startTime, RosObserver& observer) const override;

    const InformationConfig& config() const { return m_config; }

    void loadConfig(const nlohmann::json& j) override {
        m_config.informationDurationMin = j.value("information_duration_min", m_config.informationDurationMin);
        m_config.informationDurationMax = j.value("information_duration_max", m_config.informationDurationMax);
    }
    nlohmann::json saveConfig() const override {
        return {
            {"information_duration_min", m_config.informationDurationMin},
            {"information_duration_max", m_config.informationDurationMax},
        };
    }
};

inline const InformationConfig& informationConfig() {
    return static_cast<const InformationPlugin&>(
        OrderRegistry::instance().get(InformationPlugin::kTypeName)).config();
}

}  // namespace des
