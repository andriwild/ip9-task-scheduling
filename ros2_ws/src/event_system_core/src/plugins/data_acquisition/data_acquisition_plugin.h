#pragma once
#include "util/constants.h"

#include <behaviortree_cpp/bt_factory.h>
#include <string>

#include "../i_order_plugin.h"
#include "../order_registry.h"
#include "./data_acquisition_subtree.h"


namespace des {

class Scheduler;
class ISimContext;
class RosObserver;
struct DataAcquisitionConfig {
    double dataAcquisitionDuration = 120.0;
    double rewardWeight = 0.12;
    double acquisitionInterval = SECONDS_PER_DAY;
};

class DataAcquisition: public IOrderPlugin {
    DataAcquisitionConfig m_config;
public:
    static constexpr auto kTypeName = "data_acquisition";

    explicit DataAcquisition() = default;

    std::string typeName() const override { return kTypeName; }
    std::string rootSubtreeId() const override { return "DataAcquisitionRoutine"; }
    ExecutionMode executionMode() const override { return ExecutionMode::BACKGROUND; }

    void onMissionStart(ISimContext& ctx, IOrder& order) override;
    void onMissionEnd(ISimContext& ctx, IOrder& order) override;
    virtual void onStartDriveEvent(ISimContext& ctx, IOrder& order) override;
    virtual void onStopDriveEvent(ISimContext& ctx, IOrder& order) override;

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return DATA_ACQUISITION_SUBTREE_XML; }
    OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const IOrder& order, const ISimContext& context) const override;
    std::optional<std::string> targetLocation(const IOrder& order) const override;
    double estimateServiceDuration(const IOrder& order, const EstimationView& view) const override;
    double estimateReward(const IOrder& order, const EstimationView& view) const override;
    void publishTimeline(const IOrder& order, int startTime, RosObserver& observer) const override;

    const DataAcquisitionConfig& config() const { return m_config; }

    void loadConfig(const nlohmann::json& j) override {
        m_config.dataAcquisitionDuration = j.value("data_acquisition_duration", m_config.dataAcquisitionDuration);
        m_config.rewardWeight            = j.value("reward_weight", m_config.rewardWeight);
        m_config.acquisitionInterval     = j.value("data_acquisition_interval", m_config.acquisitionInterval);
    }
    nlohmann::json saveConfig() const override {
        return {
            {"data_acquisition_duration", m_config.dataAcquisitionDuration},
            {"reward_weight", m_config.rewardWeight},
            {"data_acquisition_interval", m_config.acquisitionInterval},
        };
    }
};

inline const DataAcquisitionConfig& dataAcquisitionConfig() {
    return static_cast<const DataAcquisition&>(
        OrderRegistry::instance().get(DataAcquisition::kTypeName)).config();
}

}  // namespace des
