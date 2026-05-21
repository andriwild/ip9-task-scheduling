#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "../order_registry.h"
#include "./information_subtree.h"

class Scheduler;
class ISimContext;
class RosObserver;

struct InformationConfig {
    double informationDuration = 30.0;
};

class InformationPlugin : public IOrderPlugin {
    InformationConfig m_config;
public:
    static constexpr auto kTypeName = "information";

    explicit InformationPlugin() = default;

    std::string typeName() const override { return kTypeName; }
    std::string rootSubtreeId() const override { return "InformationRoutine"; }
    des::ExecutionMode executionMode() const override { return des::ExecutionMode::INTERRUPT; }

    void onMissionStart(ISimContext& ctx, des::IOrder& order) override;
    void onMissionEnd(ISimContext& ctx, des::IOrder& order) override;
    void onStartDriveEvent(ISimContext& ctx, des::IOrder& order) override;
    void onStopDriveEvent(ISimContext& ctx, des::IOrder& order) override;

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return INFORMATION_SUBTREE_XML; }
    des::OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const des::IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const des::IOrder& order, const ISimContext& context) const override;
    void publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const override;

    const InformationConfig& config() const { return m_config; }

    void loadConfig(const nlohmann::json& j) override {
        m_config.informationDuration = j.value("information_duration", m_config.informationDuration);
    }
    nlohmann::json saveConfig() const override {
        return { {"information_duration", m_config.informationDuration} };
    }
};

inline const InformationConfig& informationConfig() {
    return static_cast<const InformationPlugin&>(
        OrderRegistry::instance().get(InformationPlugin::kTypeName)).config();
}
