#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <random>
#include <string>

#include "plugins/i_order_plugin.h"
#include "plugins/order_registry.h"
#include "util/rnd.h"
#include "accompany_subtree.h"

class Scheduler;
class ISimContext;
class RosObserver;

struct AccompanyConfig {
    double accompanySpeed             = 0.20;
    double conversationProbability    = 0.8;
    double conversationDurationMean   = 30.0;
    double conversationDurationStd    = 5.0;
    double appointmentDuration        = 1800.0;
};

class AccompanyOrderPlugin : public IOrderPlugin {
    AccompanyConfig m_config;
public:
    static constexpr auto kTypeName = "accompany";

    explicit AccompanyOrderPlugin() = default;

    std::string typeName() const override { return kTypeName; }
    std::string rootSubtreeId() const override { return "AccompanyRoutine"; }
    des::ExecutionMode executionMode() const override { return des::ExecutionMode::SCHEDULED; }

    void onMissionStart(ISimContext& ctx, des::IOrder& order) override;
    void onMissionEnd(ISimContext& ctx, des::IOrder& order) override;
    virtual void onStartDriveEvent(ISimContext& ctx, des::IOrder& order) override;
    virtual void onStopDriveEvent(ISimContext& ctx, des::IOrder& order) override;

    void registeredNodes(BT::BehaviorTreeFactory& factory) override;
    std::string subtreeXml() const override { return ACCOMPANY_SUBTREE_XML; }
    des::OrderPtr fromJson(const nlohmann::json& j) const override;
    int planDispatchTime(const des::IOrder& order, const Scheduler& scheduler, const std::string& startPos) const override;
    bool isFeasible(const des::IOrder& order, const ISimContext& context) const override;
    std::optional<std::string> targetLocation(const des::IOrder& order) const override;
    double estimateMissionEnergy(const des::IOrder& order, const ISimContext& context, const std::string& startLocation) const override;
    void publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const override;

    const AccompanyConfig& config() const { return m_config; }

    void loadConfig(const nlohmann::json& j) override {
        m_config.accompanySpeed           = j.value("accompany_speed",            m_config.accompanySpeed);
        m_config.conversationProbability  = j.value("conversation_probability",   m_config.conversationProbability);
        m_config.conversationDurationMean = j.value("conversation_duration_mean", m_config.conversationDurationMean);
        m_config.conversationDurationStd  = j.value("conversation_duration_std",  m_config.conversationDurationStd);
        m_config.appointmentDuration      = j.value("appointment_duration",       m_config.appointmentDuration);
    }
    nlohmann::json saveConfig() const override {
        return {
            {"accompany_speed",            m_config.accompanySpeed},
            {"conversation_probability",   m_config.conversationProbability},
            {"conversation_duration_mean", m_config.conversationDurationMean},
            {"conversation_duration_std",  m_config.conversationDurationStd},
            {"appointment_duration",       m_config.appointmentDuration},
        };
    }
};

inline const AccompanyConfig& accompanyConfig() {
    return static_cast<const AccompanyOrderPlugin&>(
        OrderRegistry::instance().get(AccompanyOrderPlugin::kTypeName)).config();
}

// Draw a single conversation duration. Floors at 1s so the simulation can't
// schedule a zero-length conversation event when the std puts us in the tail.
inline double rndAccompanyConversationTime(std::mt19937& rng) {
    const auto& cfg = accompanyConfig();
    const double t = rnd::normal(rng, cfg.conversationDurationMean, cfg.conversationDurationStd);
    return t < 1.0 ? 1.0 : t;
}
