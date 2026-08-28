#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <random>
#include <string>

#include "plugins/i_order_plugin.h"
#include "plugins/order_registry.h"
#include "util/rnd.h"
#include "accompany_subtree.h"

namespace des {

class Scheduler;
class ISimContext;
struct AccompanyConfig {
    double accompanySpeed             = 1.00;
    double conversationProbability    = 0.8;
    double conversationDurationMean   = 30.0;
    double conversationDurationStd    = 5.0;
    double askDurationMean            = 15.0;
    double askDurationStd             = 5.0;
    double appointmentDuration        = 1800.0;
};

class AccompanyOrderPlugin : public IOrderPlugin {
    AccompanyConfig m_config;
public:
    static constexpr auto kTypeName = "accompany";

    explicit AccompanyOrderPlugin() = default;

    std::string typeName() const override { return kTypeName; }
    std::string rootSubtreeId() const override { return "AccompanyRoutine"; }
    ExecutionMode executionMode() const override { return ExecutionMode::SCHEDULED; }

    void onMissionStart(ISimContext& ctx, IOrder& order) override;
    void onMissionResume(ISimContext& ctx, IOrder& order) override;
    void onMissionEnd(ISimContext& ctx, IOrder& order) override;
    virtual void onStartDriveEvent(ISimContext& ctx, IOrder& order) override;
    virtual void onStopDriveEvent(ISimContext& ctx, IOrder& order) override;
    void onConversationStart(ISimContext& ctx, IOrder& order, ConversationKind kind) override;

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return ACCOMPANY_SUBTREE_XML; }
    OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const IOrder& order, const ISimContext& context) const override;
    std::optional<std::string> targetLocation(const IOrder& order) const override;
    std::string outcomeDetail(const IOrder& order) const override;
    double estimateMissionEnergy(const IOrder& order, const ISimContext& context, const std::string& startLocation) const override;
    double estimateMissionDuration(const IOrder& order, const ISimContext& context, const std::string& startLocation) const override;
    void publishTimeline(const IOrder& order, int startTime, ITimelineSink& sink) const override;

    const AccompanyConfig& config() const { return m_config; }

    void loadConfig(const nlohmann::json& j) override {
        m_config.accompanySpeed           = j.value("accompany_speed",            m_config.accompanySpeed);
        m_config.conversationProbability  = j.value("conversation_probability",   m_config.conversationProbability);
        m_config.conversationDurationMean = j.value("conversation_duration_mean", m_config.conversationDurationMean);
        m_config.conversationDurationStd  = j.value("conversation_duration_std",  m_config.conversationDurationStd);
        m_config.askDurationMean          = j.value("ask_duration_mean",          m_config.askDurationMean);
        m_config.askDurationStd           = j.value("ask_duration_std",           m_config.askDurationStd);
        m_config.appointmentDuration      = j.value("appointment_duration",       m_config.appointmentDuration);
    }
    nlohmann::json saveConfig() const override {
        return {
            {"accompany_speed",            m_config.accompanySpeed},
            {"conversation_probability",   m_config.conversationProbability},
            {"conversation_duration_mean", m_config.conversationDurationMean},
            {"conversation_duration_std",  m_config.conversationDurationStd},
            {"ask_duration_mean",          m_config.askDurationMean},
            {"ask_duration_std",           m_config.askDurationStd},
            {"appointment_duration",       m_config.appointmentDuration},
        };
    }
};

inline const AccompanyConfig& accompanyConfig() {
    return static_cast<const AccompanyOrderPlugin&>(
        OrderRegistry::instance().get(AccompanyOrderPlugin::kTypeName)).config();
}

}  // namespace des
