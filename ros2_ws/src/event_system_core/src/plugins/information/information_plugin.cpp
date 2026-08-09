#include "information_plugin.h"

#include <algorithm>
#include <memory>

#include "bt_nodes/information.h"
#include "information_order.h"
#include "states.h"
#include "../../util/rnd.h"

namespace des {

void InformationPlugin::registeredNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ExecuteInformation>("ExecuteInformation");
}

void InformationPlugin::onMissionStart(ISimContext& ctx, IOrder& /*order*/) {
    ctx.changeRobotState(std::make_unique<InformationState>());
}

// pre-interrupt state is restored by popInterrupt
void InformationPlugin::onMissionEnd(ISimContext& /*ctx*/, IOrder& /*order*/) {}
void InformationPlugin::onStartDriveEvent(ISimContext& /*ctx*/, IOrder& /*order*/) {}
void InformationPlugin::onStopDriveEvent(ISimContext& /*ctx*/, IOrder& /*order*/) {}

OrderPtr InformationPlugin::fromJson(const nlohmann::json& j) const {
    auto o = std::make_shared<InformationOrder>();
    o->id          = j.at("id");
    o->type        = "information";
    o->description = j.value("description", "Information");
    o->execution   = ExecutionMode::INTERRUPT;
    return o;
}

int InformationPlugin::planDispatchTime(const IOrder& order, const Scheduler& /*s*/, const std::string& /*startPos*/) const {
    return order.dispatchTime;
}

bool InformationPlugin::isFeasible(const IOrder& /*order*/, const ISimContext& /*context*/) const {
    return true;
}

double InformationPlugin::estimateMissionDuration(const IOrder& order, const ISimContext& context, const std::string& /*startLocation*/) const {
    const auto& info = static_cast<const InformationOrder&>(order);
    if (info.sampledDuration < 0.0) {
        const auto& cfg = informationConfig();
        info.sampledDuration = std::max(1.0, rnd::uni(context.robotRng(), cfg.informationDurationMin, cfg.informationDurationMax));
    }
    return info.sampledDuration;
}

void InformationPlugin::publishTimeline(const IOrder& order, int startTime, ITimelineSink& sink) const {
    sink.publishMeeting(
        order.id,
        startTime,
        startTime,
        static_cast<int>(order.state),
        kTypeName,
        "",
        "",
        order.description,
        static_cast<int>(order.execution));
}

}  // namespace des
