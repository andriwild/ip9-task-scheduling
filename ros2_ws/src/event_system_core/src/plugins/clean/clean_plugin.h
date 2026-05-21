#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "../order_registry.h"
#include "./clean_subtree.h"


class Scheduler;
class ISimContext;
class RosObserver;

struct CleanConfig {
    double cleaningArea = 0.09;
};

class CleanPlugin: public IOrderPlugin {
    CleanConfig m_config;
public:
    static constexpr auto kTypeName = "clean";

    explicit CleanPlugin() = default;

    std::string typeName() const override { return kTypeName; }
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
    double estimateMissionEnergy(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) const override;
    double estimateMissionDuration(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) const override;
    void publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const override;

    const CleanConfig& config() const { return m_config; }

    void loadConfig(const nlohmann::json& j) override {
        m_config.cleaningArea = j.value("cleaning_area", m_config.cleaningArea);
    }
    nlohmann::json saveConfig() const override {
        return { {"cleaning_area", m_config.cleaningArea} };
    }
};

inline const CleanConfig& cleanConfig() {
    return static_cast<const CleanPlugin&>(
        OrderRegistry::instance().get(CleanPlugin::kTypeName)).config();
}
