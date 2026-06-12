#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "../order_registry.h"
#include "./data_acquisition_subtree.h"


class Scheduler;
class ISimContext;
class RosObserver;

struct DataAcquisitionConfig {
    double dataAcquisitionDuration = 120.0;
};

class DataAcquisition: public IOrderPlugin {
    DataAcquisitionConfig m_config;
public:
    static constexpr auto kTypeName = "data_acquisition";

    explicit DataAcquisition() = default;

    std::string typeName() const override { return kTypeName; }
    std::string rootSubtreeId() const override { return "DataAcquisitionRoutine"; }
    des::ExecutionMode executionMode() const override { return des::ExecutionMode::BACKGROUND; }

    void onMissionStart(ISimContext& ctx, des::IOrder& order) override;
    void onMissionEnd(ISimContext& ctx, des::IOrder& order) override;
    virtual void onStartDriveEvent(ISimContext& ctx, des::IOrder& order) override;
    virtual void onStopDriveEvent(ISimContext& ctx, des::IOrder& order) override;

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return DATA_ACQUISITION_SUBTREE_XML; }
    des::OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const des::IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const des::IOrder& order, const ISimContext& context) const override;
    std::optional<std::string> targetLocation(const des::IOrder& order) const override;
    double estimateServiceDuration(const des::IOrder& order, const ISimContext& context) const override;
    double estimateServiceEnergy(const des::IOrder& order, const ISimContext& context) const override;
    void publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const override;

    const DataAcquisitionConfig& config() const { return m_config; }

    void loadConfig(const nlohmann::json& j) override {
        m_config.dataAcquisitionDuration = j.value("data_acquisition_duration", m_config.dataAcquisitionDuration);
    }
    nlohmann::json saveConfig() const override {
        return { {"data_acquisition_duration", m_config.dataAcquisitionDuration} };
    }
};

inline const DataAcquisitionConfig& dataAcquisitionConfig() {
    return static_cast<const DataAcquisition&>(
        OrderRegistry::instance().get(DataAcquisition::kTypeName)).config();
}
