#include "accompany_plugin.h"

#include <memory>

#include "bt_nodes/search.h"
#include "bt_nodes/accompany.h"
#include "bt_nodes/conversation.h"
#include "../../sim/scheduler.h"
#include "../../model/i_sim_context.h"
#include "../../observer/ros.h"

void AccompanyOrderPlugin::registeredNodes(BT::BehaviorTreeFactory& factory) {
    // search
    factory.registerNodeType<IsSearching>("IsSearching");
    factory.registerNodeType<IsScanning>("IsScanning");
    factory.registerNodeType<FoundPerson>("FoundPerson");
    factory.registerNodeType<ScanLocation>("ScanLocation");
    factory.registerNodeType<HasNextLocation>("HasNextLocation");
    factory.registerNodeType<MoveToNextLocation>("MoveToNextLocation");
    factory.registerNodeType<StartAccompanyConversation>("StartAccompanyConversation");
    factory.registerNodeType<AbortSearch>("AbortSearch");

    // accompany
    factory.registerNodeType<IsAccompany>("IsAccompany");
    factory.registerNodeType<ArrivedWithPerson>("ArrivedWithPerson");
    factory.registerNodeType<StartDropOffConversation>("StartDropOffConversation");
    factory.registerNodeType<AbortAccompany>("AbortAccompany");

    // conversation
    factory.registerNodeType<IsConversating>("IsConversating");
    factory.registerNodeType<ConversationFinished>("ConversationFinished");
    factory.registerNodeType<WasConversationSuccessful>("WasConversationSuccessful");
    factory.registerNodeType<IsFoundPersonConversation>("IsFoundPersonConversation");
    factory.registerNodeType<IsDropOffConversation>("IsDropOffConversation");
    factory.registerNodeType<StartAccompanyAction>("StartAccompanyAction");
    factory.registerNodeType<CompleteMissionAction>("CompleteMissionAction");
}

des::OrderPtr AccompanyOrderPlugin::fromJson(const nlohmann::json& j) const {
    auto o = std::make_shared<AccompanyOrder>();
    o->id          = j.at("id");
    o->type        = "accompany";
    o->deadline    = j.at("appointmentTime");
    o->description = j.value("description", "");
    o->personName  = j.at("personName");
    o->roomName    = j.at("roomName");
    return o;
}

int AccompanyOrderPlugin::planDispatchTime(const des::IOrder& order, const Scheduler& s, const std::string& startPos) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);
    const double driveTime = s.pessimisticMeeting(a.personName, startPos, a.roomName);
    return *a.deadline - static_cast<int>(driveTime) - s.timeBuffer();
}

bool AccompanyOrderPlugin::isFeasible(const des::IOrder& order, const ISimContext& context) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);

    const auto robotLocation = context.getRobot()->getLocation();
    const double missionDuration = context.getScheduler().optimisticMeeting(a.personName, robotLocation, a.roomName);
    if (order.deadline.value() - missionDuration >= context.getTime()) {
        RCLCPP_DEBUG(rclcpp::get_logger("Context"), "Mission %u is feasible", order.id);
        return true;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("Context"), "Mission %u is NOT feasible", order.id);
    return false;
}

void AccompanyOrderPlugin::publishTimeline(const des::IOrder& order, int startTime, RosObserver& observer) const {
    const auto& a = static_cast<const AccompanyOrder&>(order);
    observer.publishMeeting(
        a.id,
        startTime,
        a.deadline.value_or(0),
        static_cast<int>(a.state),
        a.personName,
        a.description);
}
