#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>

#include "../../../util/log.h"
#include "engine/contracts/i_sim_context.h"
#include "plugins/information/events/start_information_event.h"

namespace des {

class ExecuteInformation final : public BT::StatefulActionNode {
public:
    ExecuteInformation(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() { return { BT::InputPort<int>("ctx") }; }

    BT::NodeStatus onStart() override   { return tickInternal(); }
    BT::NodeStatus onRunning() override { return tickInternal(); }
    void onHalted() override {}

private:
    BT::NodeStatus tickInternal() {
        const auto ctx = config().blackboard.get()->get<ISimContext*>("ctx");
        const auto order = ctx->getOrderPtr();
        if (!order) return BT::NodeStatus::FAILURE;

        if (order->state == MissionState::PENDING) {
            DES_LOG_DEBUG("des.plugin.information", "ExecuteInformation: start (order id=%d type=%s)", order->id, order->type.c_str());
            order->state = MissionState::IN_PROGRESS;
            ctx->pushEvent(std::make_shared<StartInformationEvent>(ctx->getTime(), order));
            return BT::NodeStatus::RUNNING;
        }
        if (order->state == MissionState::COMPLETED) {
            DES_LOG_DEBUG("des.plugin.information", "ExecuteInformation: done (order id=%d type=%s)", order->id, order->type.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
};

}  // namespace des
