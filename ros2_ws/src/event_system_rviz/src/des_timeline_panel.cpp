#include <event_system_rviz/des_timeline_panel.hpp>
#include <limits>
#include <memory>
#include <rviz_common/display_context.hpp>

#include "event_system_msgs/msg/timeline_event.hpp"
#include "event_system_msgs/msg/timeline_meeting.hpp"
#include "event_system_msgs/msg/timeline_reset.hpp"
#include "event_system_msgs/msg/timeline_state_change.hpp"


constexpr int ONE_HOUR = 3600;

namespace des_timeline_panel {
using TimelineEvent = event_system_msgs::msg::TimelineEvent;

DesTimelinePanel::DesTimelinePanel(QWidget * parent) : Panel(parent) {
    auto layout = new QVBoxLayout(this);
    m_timeline  = new Timeline();

    layout->addWidget(m_timeline);
    m_minStartTime = std::numeric_limits<int>::max();
    m_maxEndTime   = std::numeric_limits<int>::min();
}

DesTimelinePanel::~DesTimelinePanel() = default;

void DesTimelinePanel::onInitialize() {
    const auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    m_node = node_abstraction->get_raw_node();

    auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();

    m_subEvent = m_node->create_subscription<event_system_msgs::msg::TimelineEvent>(
        "/timeline/event", qos,
        [this](const event_system_msgs::msg::TimelineEvent::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { this->onEvent(msg); });
        });

    m_subMeeting = m_node->create_subscription<event_system_msgs::msg::TimelineMeeting>(
        "/timeline/meeting", qos,
        [this](const event_system_msgs::msg::TimelineMeeting::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { this->onMeeting(msg); });
        });

    m_subStateChange = m_node->create_subscription<event_system_msgs::msg::TimelineStateChange>(
        "/timeline/state_change", qos,
        [this](const event_system_msgs::msg::TimelineStateChange::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { this->onStateChange(msg); });
        });

    m_subReset = m_node->create_subscription<event_system_msgs::msg::TimelineReset>(
        "/timeline/reset", qos,
        [this](const event_system_msgs::msg::TimelineReset::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { this->onReset(msg); });
        });
}

void DesTimelinePanel::onStateChange(const event_system_msgs::msg::TimelineStateChange::SharedPtr msg){
    m_timeline->handleStateChange(msg->time, msg->state, {msg->soc, msg->capacity, msg->low_threshold});
}

void DesTimelinePanel::onMeeting(const event_system_msgs::msg::TimelineMeeting::SharedPtr& msg){
    VisualAppointment appt;
    appt.id            = msg->id;
    appt.scheduledTime = msg->appointment_time;
    appt.startTime     = msg->start_time;
    appt.primaryLabel  = msg->person_name;
    appt.description   = msg->description;

    m_minStartTime = std::min(m_minStartTime, msg->start_time);
    m_maxEndTime   = std::max(m_maxEndTime  , msg->appointment_time);

    m_timeline->setRange(m_minStartTime - ONE_HOUR, m_maxEndTime + ONE_HOUR);
    m_timeline->addMeetingPlan(appt);
}

void DesTimelinePanel::onReset(const event_system_msgs::msg::TimelineReset::SharedPtr msg){
    m_minStartTime = std::numeric_limits<int>::max();
    m_maxEndTime   = std::numeric_limits<int>::min();
    m_timeline->handleReset();
}

void DesTimelinePanel::onEvent(const event_system_msgs::msg::TimelineEvent::SharedPtr msg) {

    m_minStartTime = std::min(m_minStartTime, msg->time);
    m_maxEndTime   = std::max(m_maxEndTime  , msg->time);
    m_timeline->setRange(m_minStartTime - ONE_HOUR, m_maxEndTime + ONE_HOUR);

    m_timeline->handleEvent(
        msg->time,
        {QString::fromStdString(msg->label), des::EventType(msg->type)},
        msg->is_driving,
        msg->is_charging
    );
}

}  // namespace des_timeline_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(des_timeline_panel::DesTimelinePanel, rviz_common::Panel)
